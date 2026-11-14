"""
firebase_bridge.py - Cầu nối Firestore cho gateway.py

Chức năng:
  - Ghi trạng thái toàn nhà      -> state/<gateway_id>          (ghi đè, merge)
  - Ghi sự kiện                  -> events/<auto-id>            (thêm mới)
  - Ghi thông báo (+ FCM tuỳ chọn) -> notifications/<auto-id>   (thêm mới)
  - Ghi trạng thái gateway       -> gateways/<gateway_id>       (heartbeat định kỳ)
  - Nghe lệnh từ app             <- commands/<auto-id> (status == "pending")

Thiết kế an toàn:
  - Mọi thao tác ghi đi qua hàng đợi + thread riêng: mạng chậm/đứt KHÔNG làm chậm vòng lặp
    gateway (đặc biệt là evaluate_fire_gas_context()).
  - Thiếu thư viện / thiếu file khoá / lỗi khởi tạo: chỉ cảnh báo, gateway vẫn chạy bình thường.
  - Lệnh: claim bằng transaction (pending -> processing) nên không bao giờ chạy 2 lần,
    và lệnh quá cũ bị bỏ (expired) để tránh mở cửa nhầm sau khi gateway khởi động lại.
"""

import json
import os
import queue
import threading
import time

try:
    import firebase_admin
    from firebase_admin import credentials, firestore, messaging
    _IMPORT_ERROR = None
except ImportError as _e:   # chưa pip install firebase-admin
    firebase_admin = credentials = firestore = messaging = None
    _IMPORT_ERROR = _e

try:
    from google.cloud.firestore_v1.base_query import FieldFilter
except ImportError:
    FieldFilter = None

DEFAULT_KEY_PATH = os.environ.get("FIREBASE_KEY_PATH", "serviceAccountKey.json")

QUEUE_MAX = 500                  # quá số này thì bỏ mục cũ nhất
WRITE_TIMEOUT_SEC = 10.0
HEARTBEAT_SEC = 30.0
COMMAND_MAX_AGE_SEC = 30.0       # lệnh cũ hơn mức này bị đánh dấu "expired"

COL_STATE = "state"
COL_EVENTS = "events"
COL_NOTIFICATIONS = "notifications"
COL_GATEWAYS = "gateways"
COL_COMMANDS = "commands"


def _clean(data):
    """Ép dữ liệu về kiểu Firestore chấp nhận được (bool/int/float/str/list/dict/None)."""
    return json.loads(json.dumps(data, ensure_ascii=False, default=str))


class FirebaseBridge:
    def __init__(self, gateway_id: str, key_path: str = DEFAULT_KEY_PATH,
                 command_handler=None, fcm_topic: str = None):
        """
        command_handler(room, device, value) -> (ok: bool, message: str)
        fcm_topic: nếu đặt (vd "alerts"), notification quan trọng sẽ push FCM tới topic này.
        """
        self.gateway_id = gateway_id
        self.key_path = key_path
        self._handler = command_handler
        self.fcm_topic = fcm_topic

        self.enabled = False
        self._db = None
        self._watch = None
        self._q = queue.Queue(maxsize=QUEUE_MAX)
        self._cmd_q = queue.Queue()
        self._stop = threading.Event()
        self._stopped = False
        self._threads = []
        self._gw_info = {}
        self._info_lock = threading.Lock()
        self._dropped = 0

    # ------------------------------------------------------------------
    # Khởi động / dừng
    # ------------------------------------------------------------------
    def start(self) -> bool:
        if firebase_admin is None:
            print(f"[FIREBASE][WARN] Chưa cài firebase-admin ({_IMPORT_ERROR}). "
                  "Chạy: pip install firebase-admin. Gateway vẫn chạy không có Firebase.")
            return False
        if not os.path.isfile(self.key_path):
            print(f"[FIREBASE][WARN] Không thấy file khoá '{self.key_path}'. "
                  "Gateway vẫn chạy không có Firebase.")
            return False

        try:
            if not firebase_admin._apps:
                firebase_admin.initialize_app(credentials.Certificate(self.key_path))
            self._db = firestore.client()
        except Exception as e:
            print(f"[FIREBASE][ERROR] Khởi tạo thất bại: {e}. Gateway vẫn chạy không có Firebase.")
            return False

        self.enabled = True

        for target in (self._writer_loop, self._command_loop, self._heartbeat_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

        self._start_command_listener()
        self.update_gateway(online=True, started_at=firestore.SERVER_TIMESTAMP)
        print(f"[FIREBASE] Đã kết nối project '{self._db.project}', gateway_id='{self.gateway_id}'.")
        return True

    def stop(self):
        if not self.enabled or self._stopped:
            return
        self._stopped = True
        try:
            if self._watch is not None:
                self._watch.unsubscribe()
        except Exception:
            pass
        # Đánh dấu offline rồi xả hàng đợi trước khi thoát
        self._enqueue(("set", f"{COL_GATEWAYS}/{self.gateway_id}",
                       {"online": False, "last_seen": firestore.SERVER_TIMESTAMP}, True))
        self._stop.set()
        if self._threads:
            self._threads[0].join(timeout=5)   # writer drain
        print("[FIREBASE] Đã dừng.")

    # ------------------------------------------------------------------
    # API ghi dữ liệu (không chặn, an toàn gọi từ bất kỳ thread nào)
    # ------------------------------------------------------------------
    def publish_state(self, data: dict):
        """Ghi đè state/<gateway_id> (merge)."""
        if not self.enabled:
            return
        doc = _clean(data)
        doc["gateway_id"] = self.gateway_id
        doc["updated_at"] = firestore.SERVER_TIMESTAMP
        doc["updated_at_epoch"] = int(time.time())
        self._enqueue(("set", f"{COL_STATE}/{self.gateway_id}", doc, True))

    def add_event(self, event_type: str, payload: dict = None, level: str = "info"):
        if not self.enabled:
            return
        self._enqueue(("add", COL_EVENTS, {
            "gateway_id": self.gateway_id,
            "type": event_type,
            "level": level,
            "payload": _clean(payload or {}),
            "ts": firestore.SERVER_TIMESTAMP,
            "ts_epoch": int(time.time()),
        }))

    def add_notification(self, title: str, body: str, level: str = "info", event_type: str = None):
        if not self.enabled:
            return
        self._enqueue(("add", COL_NOTIFICATIONS, {
            "gateway_id": self.gateway_id,
            "title": title,
            "body": body,
            "level": level,
            "type": event_type,
            "read": False,
            "ts": firestore.SERVER_TIMESTAMP,
            "ts_epoch": int(time.time()),
        }))
        if self.fcm_topic and level in ("warning", "critical"):
            self._enqueue(("push", self.fcm_topic, title, body, level))

    def update_gateway(self, **fields):
        """Cập nhật gateways/<gateway_id>; các field được nhớ lại để heartbeat gửi kèm."""
        if not self.enabled:
            return
        with self._info_lock:
            self._gw_info.update(fields)
        doc = _with_sentinel(fields)
        doc["gateway_id"] = self.gateway_id
        doc["last_seen"] = firestore.SERVER_TIMESTAMP
        self._enqueue(("set", f"{COL_GATEWAYS}/{self.gateway_id}", doc, True))

    # ------------------------------------------------------------------
    # Hàng đợi ghi
    # ------------------------------------------------------------------
    def _enqueue(self, item):
        if not self.enabled:
            return
        try:
            self._q.put_nowait(item)
        except queue.Full:
            try:
                self._q.get_nowait()   # bỏ mục cũ nhất
            except queue.Empty:
                pass
            self._dropped += 1
            if self._dropped % 50 == 1:
                print(f"[FIREBASE][WARN] Hàng đợi đầy (mạng chậm?), đã bỏ {self._dropped} mục cũ.")
            try:
                self._q.put_nowait(item)
            except queue.Full:
                pass

    def _writer_loop(self):
        while True:
            try:
                item = self._q.get(timeout=0.5)
            except queue.Empty:
                if self._stop.is_set():
                    break
                continue
            self._execute(item)

    def _execute(self, item):
        kind = item[0]
        try:
            if kind == "set":
                _, path, data, merge = item
                self._db.document(path).set(data, merge=merge, timeout=WRITE_TIMEOUT_SEC)
            elif kind == "add":
                _, collection, data = item
                self._db.collection(collection).add(data, timeout=WRITE_TIMEOUT_SEC)
            elif kind == "push":
                _, topic, title, body, level = item
                messaging.send(messaging.Message(
                    topic=topic,
                    notification=messaging.Notification(title=title, body=body),
                    data={"level": level, "gateway_id": self.gateway_id},
                    android=messaging.AndroidConfig(priority="high"),
                ))
        except Exception as e:
            print(f"[FIREBASE][WARN] Ghi '{kind}' thất bại: {e}")

    def _heartbeat_loop(self):
        while not self._stop.wait(HEARTBEAT_SEC):
            with self._info_lock:
                info = {k: v for k, v in self._gw_info.items() if k != "started_at"}
            info["online"] = True
            info["gateway_id"] = self.gateway_id
            info["last_seen"] = firestore.SERVER_TIMESTAMP
            self._enqueue(("set", f"{COL_GATEWAYS}/{self.gateway_id}", _with_sentinel(info), True))

    # ------------------------------------------------------------------
    # Nghe lệnh từ app
    # ------------------------------------------------------------------
    def _start_command_listener(self):
        coll = self._db.collection(COL_COMMANDS)
        if FieldFilter is not None:
            query = (coll.where(filter=FieldFilter("gateway_id", "==", self.gateway_id))
                         .where(filter=FieldFilter("status", "==", "pending")))
        else:
            query = coll.where("gateway_id", "==", self.gateway_id).where("status", "==", "pending")
        self._watch = query.on_snapshot(self._on_commands_snapshot)

    def _on_commands_snapshot(self, col_snapshot, changes, read_time):
        # Chạy ở thread của Firestore: chỉ đẩy tham chiếu vào hàng đợi, xử lý ở thread riêng
        for change in changes:
            if change.type.name == "ADDED":
                self._cmd_q.put(change.document.reference)

    def _command_loop(self):
        while not self._stop.is_set():
            try:
                ref = self._cmd_q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process_command(ref)
            except Exception as e:
                print(f"[FIREBASE][ERROR] Xử lý lệnh lỗi: {e}")

    def _claim(self, ref):
        """pending -> processing trong transaction. Trả về dict lệnh, hoặc None nếu đã bị lấy/không còn pending."""
        @firestore.transactional
        def _txn(transaction, ref):
            snap = ref.get(transaction=transaction)
            data = snap.to_dict() if snap.exists else None
            if not data or data.get("status") != "pending":
                return None
            transaction.update(ref, {"status": "processing", "claimed_at": firestore.SERVER_TIMESTAMP})
            return data
        return _txn(self._db.transaction(), ref)

    def _finish(self, ref, status: str, message: str):
        try:
            ref.update({
                "status": status,
                "result": message,
                "processed_at": firestore.SERVER_TIMESTAMP,
            }, timeout=WRITE_TIMEOUT_SEC)
        except Exception as e:
            print(f"[FIREBASE][WARN] Không cập nhật được trạng thái lệnh: {e}")

    def _process_command(self, ref):
        data = self._claim(ref)
        if data is None:
            return

        # Bỏ lệnh quá cũ (vd gateway vừa khởi động lại, tránh mở cửa nhầm)
        created = data.get("created_at")
        if hasattr(created, "timestamp"):
            age = time.time() - created.timestamp()
            if age > COMMAND_MAX_AGE_SEC:
                print(f"[FIREBASE] Bỏ lệnh quá cũ ({age:.0f}s): {data}")
                self._finish(ref, "expired", f"Lệnh quá cũ ({age:.0f}s)")
                return

        room = str(data.get("room", "")).strip()
        device = str(data.get("device", "")).strip()
        value = str(data.get("value", "")).strip()
        if not room or not device or value == "":
            self._finish(ref, "failed", "Thiếu room/device/value")
            return

        if self._handler is None:
            self._finish(ref, "failed", "Gateway chưa cấu hình xử lý lệnh")
            return

        print(f"[FIREBASE] Nhận lệnh: {room}/{device} = {value}")
        try:
            ok, message = self._handler(room, device, value)
        except Exception as e:
            ok, message = False, f"Lỗi xử lý: {e}"
        self._finish(ref, "done" if ok else "failed", message)


def _with_sentinel(d: dict) -> dict:
    """Giữ nguyên các sentinel của Firestore (SERVER_TIMESTAMP), làm sạch phần còn lại."""
    out = {}
    for k, v in d.items():
        out[k] = v if k in ("last_seen", "started_at") else _clean(v)
    return out
