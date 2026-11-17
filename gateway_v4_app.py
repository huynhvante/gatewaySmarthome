import serial
import json
import re
import time
import os
import signal
import sys
import threading
from datetime import datetime

import paho.mqtt.client as mqtt

import living_room
from firebase_bridge import FirebaseBridge

# ==========================================
# CẤU HÌNH
# ==========================================
PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

ENABLE_FILE_LOG = False
LOG_PATH = "sensor_log.jsonl"

MQTT_HOST = "749e4310b50e423c8876f553789253da.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "hvt"
# Không để mật khẩu trong code. Chạy: export MQTT_PASSWORD='...'
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD", "")
MQTT_CLIENT_ID = "raspi-home-gateway"

# --- Firebase (Firestore) ---
# File khoá đọc từ biến môi trường FIREBASE_KEY_PATH (mặc định: serviceAccountKey.json cạnh gateway.py)
FIREBASE_GATEWAY_ID = MQTT_CLIENT_ID
FIREBASE_STATE_INTERVAL_SEC = 15.0      # chu kỳ ghi state/<gateway_id>
FIREBASE_FCM_TOPIC = None               # đặt "alerts" nếu app Android đã subscribe topic FCM

# --- Topic điều khiển / trạng thái nội bộ (giữ nguyên) ---
TOPIC_DOOR_CMD = "home/livingroom/door/cmd"
TOPIC_FACE_UNLOCK = "home/livingroom/door/face_unlock"
TOPIC_DOOR_STATUS = "home/livingroom/door/status"
TOPIC_ALARM = "home/livingroom/alarm"
TOPIC_FIRE_GAS = "home/fire_gas/alarm"
TOPIC_FIRE_GAS_ACK = "home/fire_gas/ack"
TOPIC_HOUSE_STATUS = "home/gateway/status"

# Topic cho app Android: smarthome/<room>/<device>/<metric>, payload là chuỗi thuần
ROOT_TOPIC = "smarthome"
APP_PUBLISH_INTERVAL_SEC = 5.0

# Trạng thái bridge (cổng serial) báo cho app
TOPIC_BRIDGE_STATE = f"{ROOT_TOPIC}/gateway/bridge/state"   # "ONLINE" | "OFFLINE" (retained)
TOPIC_BRIDGE_ERROR = f"{ROOT_TOPIC}/gateway/bridge/error"   # nội dung lỗi (retained), rỗng khi bình thường
TOPIC_GATEWAY_ERROR = f"{ROOT_TOPIC}/gateway/error"         # lỗi tức thời, vd lệnh không gửi được (không retained)
BRIDGE_RETRY_SEC = 5.0                                       # chu kỳ tự thử mở lại cổng serial
_BRIDGE_IMPACT = "Dữ liệu bếp/phòng ngủ và cảnh báo gas/cháy của 2 phòng này tạm thời không hoạt động."

CONTROLLABLE_DEVICES = [
    ("living_room", "led"),
    ("living_room", "fan"),
    ("living_room", "door"),
    ("bedroom", "led"),
    ("bedroom", "fan"),
    ("bedroom", "buzzer"),
    ("kitchen", "led"),
    ("kitchen", "fan"),
    ("kitchen", "gas_threshold"),
]
TOPIC_APP_CMD_SET = {
    f"{ROOT_TOPIC}/{room}/{device}/set": (room, device)
    for room, device in CONTROLLABLE_DEVICES
}

# Ngưỡng gas đặt qua app: smarthome/kitchen/gas_threshold/set, payload là số nguyên 50..3000.
# Gateway chuyển xuống node kitchen bằng lệnh "kitchen:GAS_TH=<số>\n".
# Giá trị đang dùng thật sự là giá trị node kitchen báo lại trong telemetry (state["kit_gas_threshold"]).
# Khoảng này phải khớp GAS_TH_MIN / GAS_TH_MAX trong firmware kitchen.
# Báo gas chỉ so gas_raw với ngưỡng này (không dùng điện áp).
GAS_THRESHOLD_MIN = 50
GAS_THRESHOLD_MAX = 3000
_GAS_TH_RE = re.compile(r"^GAS_TH=(\d{1,4})$")

TEMP_FIRE_THRESHOLD = 55.0
TEMP_RISE_THRESHOLD = 3.0
TEMP_RISE_WINDOW_SEC = 10.0
UNOCCUPIED_AFTER_SEC = 30 * 60
NIGHT_START_HOUR = 23
NIGHT_END_HOUR = 6
NIGHT_UNLOCK_AFTER_SEC = 30
DAY_ESCALATE_AFTER_SEC = 60
COMMAND_REPEAT_SEC = 10

# Hết nguy hiểm phải liên tục bấy lâu mới tắt báo động (tránh còi kêu/tắt nhấp nháy khi gas dao động quanh ngưỡng)
CLEAR_HOLD_SEC = 10
# Số lần gửi lệnh tắt còi (ESP-NOW không có xác nhận, gửi lặp để tránh mất gói)
ALARM_OFF_REPEAT = 3
ALARM_OFF_REPEAT_GAP_SEC = 0.3

# Sau khi app gửi lệnh đèn/quạt, bỏ qua telemetry cũ chừng này giây (chờ node cập nhật xong)
DEVICE_SYNC_GRACE_SEC = 4.0

# ==========================================
# TRẠNG THÁI TOÀN CỤC
# ==========================================
state = {
    "kit_temp": None,
    "kit_humid": None,
    "kit_gas_raw": None,
    "kit_gas_voltage": None,     # chỉ để hiển thị / báo cáo, không dùng để quyết định báo động
    "kit_gas_threshold": None,   # None cho tới khi node kitchen báo lại
    "kit_fan": False,
    "kit_led": False,
    "kit_alarm": False,

    "bed_temp": None,
    "bed_humid": None,
    "bed_light": None,
    "bed_motion": False,
    "bed_fan": False,
    "bed_led": False,
    "bed_alarm_level": 0,
    "fire_gas_context": "clear",
    "fire_gas_level": "CLEAR",
    "fire_gas_reason": "none",
}

ser = None
_log_fh = None
_shutdown = False
_living_room_thread = None
_kit_temp_history = []
_danger_started_ts = None
_safe_since = None         # thời điểm bắt đầu hết nguy hiểm (để giữ báo động thêm CLEAR_HOLD_SEC)
_day_escalated = False
_unlock_sent = False
_manual_ack_ts = 0.0
_last_presence_ts = time.time()
_last_command = {}
_mqtt_client = None
_last_gas_alert = None
_device_states = {}
_device_cmd_ts = {}        # (room, device) -> thời điểm app gửi lệnh gần nhất
_last_fb_alarm = None      # (context, level) đã ghi lên Firebase, tránh ghi lặp mỗi giây
_last_lr_alarm_key = {}    # event_type -> (level, context) đã phát từ phòng khách, tránh phát lặp mỗi giây

# Trạng thái bridge: None = chưa biết, True/False sau lần thử mở cổng đầu tiên
_bridge_online = None
_bridge_error = ""


# ==========================================
# GHI LOG AN TOÀN
# ==========================================
def safe_append_log(record: dict):
    global _log_fh
    if not ENABLE_FILE_LOG:
        return
    try:
        if _log_fh is None:
            _log_fh = open(LOG_PATH, "a", buffering=1)
        line = json.dumps(record, ensure_ascii=False)
        _log_fh.write(line + "\n")
        _log_fh.flush()
        os.fsync(_log_fh.fileno())
    except Exception as e:
        print(f"[WARN] Ghi log thất bại: {e}")


# hàm publish MQTT, payload là dict, sẽ json.dumps trước khi gửi
def _publish(topic: str, payload: dict, retain: bool = False):
    if _mqtt_client is None:
        return
    try:
        _mqtt_client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=retain)
    except Exception as e:
        print(f"[WARN] MQTT publish lỗi: {e}")


# Publish chuỗi thuần cho app Android
def _publish_raw(topic: str, value, qos: int = 0, retain: bool = False):
    if _mqtt_client is None:
        return
    try:
        _mqtt_client.publish(topic, str(value), qos=qos, retain=retain)
    except Exception as e:
        print(f"[WARN] MQTT publish lỗi: {e}")


def _pub_sensor(room: str, device: str, metric: str, value):
    if value is None:
        return
    if isinstance(value, bool):
        value = int(value)
    _publish_raw(f"{ROOT_TOPIC}/{room}/{device}/{metric}", value)


# Publish lại trạng thái cho app: smarthome/<room>/<device>/state (retained)
def _publish_device_state(room: str, device: str, value: str):
    _publish_raw(f"{ROOT_TOPIC}/{room}/{device}/state", value, qos=1, retain=True)


# Đồng bộ trạng thái khoá cửa THẬT lên app (khoá tự lại sau vài giây, app cần biết)
def _publish_door_state_to_app():
    lr = living_room.state
    value = "OFF" if lr.get("door_locked") else "ON"   # quy ước: ON = mở khoá, OFF = đã khoá
    if _device_states.get(("living_room", "door")) == value:
        return
    _device_states[("living_room", "door")] = value
    _publish_device_state("living_room", "door", value)


# Đồng bộ ngưỡng gas THẬT (do node kitchen báo lại) lên app
def _publish_gas_threshold_state_to_app():
    th = state.get("kit_gas_threshold")
    if th is None:
        return
    value = str(th)
    if _device_states.get(("kitchen", "gas_threshold")) == value:
        return
    _device_states[("kitchen", "gas_threshold")] = value
    _publish_device_state("kitchen", "gas_threshold", value)


# Đồng bộ trạng thái THẬT của đèn/quạt (từ node/phòng khách) lên app, chỉ publish khi đổi.
# Bắt được mọi nguồn thay đổi: nút vật lý, báo động, khôi phục, auto, lệnh bị mất gói...
def _sync_device_states_from_nodes():
    now = time.time()
    lr = living_room.state

    actual = {
        ("living_room", "led"): lr.get("led"),
        ("living_room", "fan"): lr.get("fan"),
    }

    # Bếp/phòng ngủ lấy qua bridge: mất bridge hoặc chưa có telemetry thì KHÔNG publish giá trị cũ
    if _bridge_online:
        if state["kit_gas_raw"] is not None:
            actual[("kitchen", "led")] = state["kit_led"]
            actual[("kitchen", "fan")] = state["kit_fan"]
        if state["bed_light"] is not None:
            actual[("bedroom", "led")] = state["bed_led"]
            actual[("bedroom", "fan")] = state["bed_fan"]

    for key, is_on in actual.items():
        if is_on is None:
            continue
        # Vừa có lệnh từ app: chờ node cập nhật, tránh app bị nhấp nháy ON -> OFF -> ON
        if now - _device_cmd_ts.get(key, 0.0) < DEVICE_SYNC_GRACE_SEC:
            continue
        value = "ON" if is_on else "OFF"
        if _device_states.get(key) == value:
            continue
        _device_states[key] = value
        _publish_device_state(key[0], key[1], value)


# Gửi toàn bộ cảm biến theo đúng cấu trúc topic của app
def publish_sensors_to_app():
    lr = living_room.state

    # Living room (không phụ thuộc bridge)
    _pub_sensor("living_room", "pir", "motion", lr.get("motion"))
    _pub_sensor("living_room", "dht", "temperature", lr.get("temp"))
    _pub_sensor("living_room", "dht", "humidity", lr.get("humid"))
    _pub_sensor("living_room", "reed", "open", lr.get("door_open"))

    # Bếp/phòng ngủ lấy qua bridge: mất bridge thì KHÔNG publish giá trị cũ như thể còn mới
    if not _bridge_online:
        return

    # Kitchen
    _pub_sensor("kitchen", "gas_sensor", "value", state["kit_gas_raw"])
    _pub_sensor("kitchen", "dht", "temperature", state["kit_temp"])
    _pub_sensor("kitchen", "dht", "humidity", state["kit_humid"])

    # Bedroom
    if state["bed_light"] is not None:
        _pub_sensor("bedroom", "pir", "motion", state["bed_motion"])
        _pub_sensor("bedroom", "dht", "temperature", state["bed_temp"])
        _pub_sensor("bedroom", "dht", "humidity", state["bed_humid"])
        _pub_sensor("bedroom", "light_sensor", "lux", state["bed_light"])


# Alert gas: chỉ publish khi đổi trạng thái, retained để app mở lên vẫn thấy
def _publish_gas_alert(active: bool):
    global _last_gas_alert
    if active == _last_gas_alert:
        return
    _last_gas_alert = active
    _publish_raw(
        f"{ROOT_TOPIC}/kitchen/gas_sensor/alert",
        "1" if active else "0",
        qos=1,
        retain=True,
    )


# Chuẩn hoá payload lệnh: nhiều app gửi ON/OFF, 1/0, true/false...
def _normalize_switch_payload(raw: str):
    v = raw.strip().upper()
    if v in ("ON", "1", "TRUE", "OPEN", "UNLOCK"):
        return "ON"
    if v in ("OFF", "0", "FALSE", "CLOSE", "LOCK"):
        return "OFF"
    return None


# Trả về chuỗi trạng thái thay vì True/False:
#   "ok"          = đã gửi lệnh tới phần cứng
#   "failed"      = có phần cứng nhưng không gửi được (bridge không kết nối)
#   "unsupported" = chưa nối phần cứng thật
def _dispatch_hardware_command(room: str, device: str, is_on: bool) -> str:
    if room in ("kitchen", "bedroom") and device in ("fan", "led"):
        cmd = f"{device.upper()}_{'ON' if is_on else 'OFF'}"      # FAN_ON, FAN_OFF, LED_ON, LED_OFF
        return "ok" if send_command(room, cmd, force=True) else "failed"
    if room == "living_room" and device == "door":
        # Quy ước: ON = mở khoá, OFF = khoá lại
        # Servo quay mất ~0.6 s nên chạy trong thread riêng, không chặn luồng MQTT
        if is_on:
            threading.Thread(target=living_room.unlock_door, kwargs={"source": "app"}, daemon=True).start()
        else:
            threading.Thread(target=living_room.lock_door, daemon=True).start()
        return "ok"
    if room == "living_room" and device == "fan":
        living_room.set_fan(is_on, source="app")
        return "ok"
    if room == "living_room" and device == "led":
        living_room.set_led(is_on, source="app")
        return "ok"
    return "unsupported"


# Nhận ngưỡng gas từ app và chuyển xuống node kitchen qua bridge.
# Trả về (ok, message): MQTT bỏ qua giá trị trả về, Firestore dùng để ghi status done/failed.
def _handle_gas_threshold_command(raw_value: str):
    try:
        th = int(float(raw_value.strip()))
    except (ValueError, OverflowError):   # OverflowError: payload như "inf" hoặc "1e999"
        msg = (f"Ngưỡng gas không hợp lệ: '{raw_value}' (cần số nguyên "
               f"{GAS_THRESHOLD_MIN}..{GAS_THRESHOLD_MAX}).")
        _notify_gateway_error(msg)
        return False, msg

    if not GAS_THRESHOLD_MIN <= th <= GAS_THRESHOLD_MAX:
        msg = f"Ngưỡng gas {th} ngoài khoảng {GAS_THRESHOLD_MIN}..{GAS_THRESHOLD_MAX}."
        _notify_gateway_error(msg)
        return False, msg

    if not send_command("kitchen", f"GAS_TH={th}", force=True):
        msg = f"Không gửi được ngưỡng gas {th}: bridge chưa kết nối."
        _notify_gateway_error(msg)
        return False, msg

    # KHÔNG publish state ở đây: chờ node kitchen báo lại trong telemetry (xem parse_data),
    # để app luôn thấy giá trị node thật sự đang dùng.
    print(f"[CMD] Đã gửi ngưỡng gas {th} xuống kitchen, chờ node xác nhận.")
    return True, f"Đã gửi ngưỡng gas {th}, chờ node xác nhận."


# Xử lý lệnh app publish vào smarthome/<room>/<device>/set (hoặc lệnh từ Firestore).
# Trả về (ok, message).
def _handle_app_command(room: str, device: str, raw_value: str):
    # Ngưỡng gas là số, không phải ON/OFF nên xử lý riêng
    if (room, device) == ("kitchen", "gas_threshold"):
        return _handle_gas_threshold_command(raw_value)

    # Đang báo gas/cháy: các node bỏ qua lệnh đèn/quạt (đèn + quạt phải giữ bật).
    # Không nhận lệnh ở đây và đẩy lại trạng thái thật lên app để nút trên app trở về đúng.
    if device in ("fan", "led") and state["fire_gas_level"] != "CLEAR":
        msg = f"Đang báo gas/cháy, bỏ qua lệnh {room}/{device}."
        _device_states.pop((room, device), None)   # buộc lần đồng bộ kế tiếp publish lại trạng thái thật
        _device_cmd_ts.pop((room, device), None)
        _notify_gateway_error(msg)
        return False, msg

    value = _normalize_switch_payload(raw_value)
    if value is None:
        msg = f"Giá trị lệnh không hợp lệ: {room}/{device} = {raw_value}"
        print(f"[WARN] {msg}")
        return False, msg

    result = _dispatch_hardware_command(room, device, value == "ON")

    # Lệnh không tới được phần cứng: báo lỗi cho app, KHÔNG xác nhận trạng thái giả
    if result == "failed":
        msg = f"Không gửi được lệnh {room}/{device} = {value}: bridge chưa kết nối."
        _notify_gateway_error(msg)
        return False, msg

    _device_cmd_ts[(room, device)] = time.time()   # cho node thời gian cập nhật trước khi đồng bộ lại
    _device_states[(room, device)] = value
    _publish_device_state(room, device, value)

    print(f"🎮 Nhận lệnh: {ROOT_TOPIC}/{room}/{device}/set = {value}  →  "
          f"đã cập nhật {ROOT_TOPIC}/{room}/{device}/state")
    if result == "unsupported":
        print(f"    (lưu ý: {room}/{device} chưa nối phần cứng thật, chỉ echo trạng thái cho app)")
        return True, "Chưa nối phần cứng thật, chỉ cập nhật trạng thái"
    return True, "ok"


# ==========================================
# FIREBASE (FIRESTORE)
# ==========================================
# Lệnh từ collection "commands": chỉ chấp nhận đúng các thiết bị điều khiển được
def _firebase_command_handler(room: str, device: str, value: str):
    if (room, device) not in CONTROLLABLE_DEVICES:
        return False, f"Thiết bị không được hỗ trợ: {room}/{device}"
    return _handle_app_command(room, device, value)


# Ảnh chụp trạng thái toàn nhà để ghi vào state/<gateway_id>
def _firebase_state_snapshot() -> dict:
    lr = living_room.state
    return {
        "kitchen": {
            "temp": state["kit_temp"],
            "humid": state["kit_humid"],
            "gas_raw": state["kit_gas_raw"],
            "gas_voltage": state["kit_gas_voltage"],
            "gas_threshold": state["kit_gas_threshold"],
            "fan": state["kit_fan"],
            "led": state["kit_led"],
            "alarm": state["kit_alarm"],
        },
        "bedroom": {
            "temp": state["bed_temp"],
            "humid": state["bed_humid"],
            "light": state["bed_light"],
            "motion": state["bed_motion"],
            "fan": state["bed_fan"],
            "led": state["bed_led"],
            "alarm_level": state["bed_alarm_level"],
        },
        "living_room": {
            "temp": lr.get("temp"),
            "humid": lr.get("humid"),
            "motion": lr.get("motion"),
            "fan": lr.get("fan"),
            "led": lr.get("led"),
            "door_open": lr.get("door_open"),
            "door_locked": lr.get("door_locked"),
            "yolo_people_count": int(lr.get("yolo_people_count") or 0),
        },
        "fire_gas": {
            "level": state["fire_gas_level"],
            "context": state["fire_gas_context"],
            "reason": state["fire_gas_reason"],
        },
        "bridge": {"online": bool(_bridge_online), "error": _bridge_error},
        "devices": {f"{r}_{d}": v for (r, d), v in _device_states.items()},
    }


fb = FirebaseBridge(
    gateway_id=FIREBASE_GATEWAY_ID,
    command_handler=_firebase_command_handler,
    fcm_topic=FIREBASE_FCM_TOPIC,
)

_LEVEL_TITLE = {
    "MED": "Cảnh báo gas/cháy",
    "MAX": "NGUY HIỂM: gas/cháy",
    "RED": "KHẨN CẤP: gas/cháy",
}


# Ghi sự kiện + thông báo khi báo động gas/cháy đổi (context, level)
def _notify_fire_gas_firebase(context: str, level: str):
    severity = "critical" if level in ("MAX", "RED") else "warning"
    payload = {**_alarm_details(), "level": level, "context": context}
    fb.add_event("fire_gas", payload, level=severity)
    fb.add_notification(
        _LEVEL_TITLE.get(level, "Cảnh báo gas/cháy"),
        f"Nguyên nhân: {state['fire_gas_reason']} (tình huống: {context})",
        level=severity,
        event_type="fire_gas",
    )


# Phòng khách phát sự kiện alarm/fire_gas mỗi lần gateway gọi trigger (nhiều lần mỗi giây khi đang báo).
# Chỉ cho đi tiếp khi (level, context) đổi, tránh spam MQTT/Firestore.
def _lr_alarm_event_changed(event_type: str, payload: dict) -> bool:
    key = (payload.get("level"), payload.get("context"))
    if _last_lr_alarm_key.get(event_type) == key:
        return False
    _last_lr_alarm_key[event_type] = key
    return True


def _on_living_room_event(event_type: str, payload: dict):
    if event_type == "door_status":
        _publish(TOPIC_DOOR_STATUS, payload, retain=True)
        _publish_door_state_to_app()      # cập nhật nút cửa trên app theo trạng thái thật
        fb.add_event("door_status", payload)
    elif event_type == "alarm":
        if not _lr_alarm_event_changed("alarm", payload):
            return
        _publish(TOPIC_ALARM, payload)
        fb.add_event("alarm", payload, level="warning")
    elif event_type == "fire_gas":
        if payload.get("level") == "CLEAR":
            _last_lr_alarm_key.clear()     # hết báo: lần báo sau (kể cả cùng mức) phải được phát lại
        elif not _lr_alarm_event_changed("fire_gas", payload):
            return
        _publish(TOPIC_FIRE_GAS, payload, retain=True)
        # Sự kiện fire_gas cho Firebase đã được ghi ở _set_fire_gas_alarm (chỉ khi đổi mức)
    elif event_type == "sensor_reading":
        safe_append_log({"ts": datetime.now().strftime("%H:%M:%S"), "node": "living_room", **payload})
    elif event_type == "device_state":
        # Nút nhấn vật lý đổi trạng thái đèn/quạt phòng khách: đồng bộ lên app
        dev = payload.get("device")
        if dev in ("led", "fan"):
            value = "ON" if payload.get("on") else "OFF"
            _device_states[("living_room", dev)] = value
            _publish_device_state("living_room", dev, value)


def _on_mqtt_connect(client, userdata, flags, rc, properties=None):
    print(f"[MQTT] Gateway đã kết nối HiveMQ, rc={rc}")
    client.subscribe(TOPIC_DOOR_CMD)
    client.subscribe(TOPIC_FACE_UNLOCK)
    client.subscribe(TOPIC_FIRE_GAS_ACK)
    for topic in TOPIC_APP_CMD_SET:
        client.subscribe(topic)
    client.publish(TOPIC_HOUSE_STATUS, "online", qos=1, retain=True)
    _publish_bridge_state()   # bridge có thể đã lỗi từ trước khi MQTT kết nối xong


def _on_mqtt_message(client, userdata, msg):
    if msg.topic in TOPIC_APP_CMD_SET:
        room, device = TOPIC_APP_CMD_SET[msg.topic]
        raw_value = msg.payload.decode("utf-8", errors="replace")
        _handle_app_command(room, device, raw_value)
        return

    payload = msg.payload.decode("utf-8", errors="replace").strip().upper()
    print(f"[MQTT] {msg.topic}: {payload}")

    if msg.topic == TOPIC_DOOR_CMD:
        if payload in ("OPEN", "UNLOCK"):
            threading.Thread(target=living_room.unlock_door, kwargs={"source": "app"}, daemon=True).start()
        elif payload in ("CLOSE", "LOCK"):
            threading.Thread(target=living_room.lock_door, daemon=True).start()
        else:
            print(f"[WARN] Lệnh cửa không hợp lệ: {payload}")

    elif msg.topic == TOPIC_FACE_UNLOCK:
        if payload in ("OK", "UNLOCK", "SUCCESS"):
            threading.Thread(target=living_room.unlock_door, kwargs={"source": "face"}, daemon=True).start()

    elif msg.topic == TOPIC_FIRE_GAS_ACK:
        if payload in ("ACK", "OK", "HANDLED", "CLEAR"):
            ack_fire_gas()


# Tương thích paho-mqtt 1.x và 2.x, tự reconnect, có LWT
def setup_mqtt():
    global _mqtt_client
    if not MQTT_PASSWORD:
        print("[WARN] MQTT_PASSWORD chưa được đặt (export MQTT_PASSWORD='...'), kết nối HiveMQ sẽ thất bại.")

    if hasattr(mqtt, "CallbackAPIVersion"):
        _mqtt_client = mqtt.Client(
            client_id=MQTT_CLIENT_ID,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
    else:
        _mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)

    _mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    _mqtt_client.tls_set()
    _mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
    _mqtt_client.will_set(TOPIC_HOUSE_STATUS, "offline", qos=1, retain=True)
    _mqtt_client.on_connect = _on_mqtt_connect
    _mqtt_client.on_message = _on_mqtt_message
    _mqtt_client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=60)
    _mqtt_client.loop_start()


# Đúng thứ tự: publish offline -> disconnect -> loop_stop
def cleanup_mqtt():
    global _mqtt_client
    if _mqtt_client is None:
        return
    try:
        info = _mqtt_client.publish(TOPIC_HOUSE_STATUS, "offline", qos=1, retain=True)
        info.wait_for_publish(timeout=2)
    except Exception:
        pass
    try:
        _mqtt_client.disconnect()
        _mqtt_client.loop_stop()
        print("[INFO] Đã ngắt HiveMQ ở gateway.")
    except Exception:
        pass
    _mqtt_client = None


# ==========================================
# BRIDGE (CỔNG SERIAL): MỞ / MẤT KẾT NỐI / BÁO LỖI CHO APP
# ==========================================
def _notify_gateway_error(message: str):
    """Báo lỗi tức thời cho app (không retained: chỉ app đang mở mới nhận)."""
    print(f"[ERROR] {message}")
    _publish_raw(TOPIC_GATEWAY_ERROR, message, qos=1)


def _publish_bridge_state():
    """Gửi trạng thái bridge cho app. Retained để app mở lên sau vẫn thấy trạng thái hiện tại."""
    if _bridge_online is None:
        return
    _publish_raw(TOPIC_BRIDGE_STATE, "ONLINE" if _bridge_online else "OFFLINE", qos=1, retain=True)
    # Payload rỗng + retained = xoá thông báo lỗi cũ khi bridge đã hoạt động lại
    _publish_raw(TOPIC_BRIDGE_ERROR, "" if _bridge_online else _bridge_error, qos=1, retain=True)


def _set_bridge_state(online: bool, reason: str = ""):
    """Đổi trạng thái bridge; chỉ log/publish khi có thay đổi (tránh lặp mỗi lần thử lại)."""
    global _bridge_online, _bridge_error
    if _bridge_online == online and _bridge_error == reason:
        return
    _bridge_online = online
    _bridge_error = reason
    if online:
        print(f"[BRIDGE] Đã kết nối {PORT} @ {BAUD_RATE} baud.")
    else:
        print(f"[BRIDGE][ERROR] {reason}")
    _publish_bridge_state()

    # Firebase: cập nhật gateways/<id>, ghi sự kiện, và thông báo khi mất bridge
    fb.update_gateway(bridge_online=online, bridge_error=reason)
    if online:
        fb.add_event("bridge_online", {"port": PORT})
    else:
        fb.add_event("bridge_offline", {"reason": reason}, level="warning")
        fb.add_notification("Mất kết nối bridge", reason, level="warning", event_type="bridge_offline")


def _open_serial() -> bool:
    """Thử mở cổng serial. Lỗi thì KHÔNG thoát gateway, chỉ báo cho app và để vòng lặp thử lại."""
    global ser
    try:
        port = serial.Serial(PORT, BAUD_RATE, timeout=1)
    except (serial.SerialException, OSError) as e:
        ser = None
        _set_bridge_state(False, f"Không kết nối được bridge ({PORT}): {e}. {_BRIDGE_IMPACT}")
        return False

    time.sleep(2)   # chờ ESP32 reset sau khi mở cổng
    try:
        port.reset_input_buffer()
    except (serial.SerialException, OSError):
        pass
    ser = port
    _set_bridge_state(True)
    return True


def _handle_bridge_lost(err):
    """Bridge bị rút / lỗi I/O khi đang chạy: đóng cổng, báo app, vòng lặp chính sẽ tự thử lại."""
    global ser
    port, ser = ser, None
    if port is not None:
        try:
            port.close()
        except Exception:
            pass
    _set_bridge_state(False, f"Mất kết nối bridge ({PORT}): {err}. {_BRIDGE_IMPACT}")


# ==========================================
# LỆNH ĐIỀU KHIỂN QUA BRIDGE
# ==========================================
def send_command(target: str, cmd: str, force: bool = False) -> bool:
    """Gửi lệnh xuống node qua bridge.
    True  = đã gửi (hoặc bỏ qua vì vừa gửi đúng lệnh này gần đây).
    False = không gửi được (lệnh sai, hoặc bridge không kết nối)."""
    allowed_targets = {"kitchen", "bedroom", "all"}
    allowed_cmds = {
        "FAN_ON", "FAN_OFF", "LED_ON", "LED_OFF",
        "ALERT_ON", "ALERT_OFF",
        "ALARM_MED", "ALARM_MAX", "ALARM_OFF",
        "ON", "OFF", "AUTO",
    }
    target = target.strip().lower()
    cmd = cmd.strip().upper()

    # Lệnh đặt ngưỡng gas: GAS_TH=<GAS_THRESHOLD_MIN..GAS_THRESHOLD_MAX>, chỉ dành cho kitchen
    gas_match = _GAS_TH_RE.match(cmd)
    valid_gas_cmd = bool(
        gas_match
        and target == "kitchen"
        and GAS_THRESHOLD_MIN <= int(gas_match.group(1)) <= GAS_THRESHOLD_MAX
    )

    if target not in allowed_targets or not (cmd in allowed_cmds or valid_gas_cmd):
        print(f"[WARN] Lệnh không hợp lệ: {target}:{cmd}")
        return False

    now = time.time()
    key = (target, cmd)
    if not force and now - _last_command.get(key, 0.0) < COMMAND_REPEAT_SEC:
        return True

    port = ser
    if port is None or not port.is_open:
        # Lệnh tự động (báo cháy...) im lặng bỏ qua để không spam log; trạng thái bridge đã được báo riêng
        if force:
            print(f"[WARN] Bridge chưa kết nối, bỏ qua lệnh: {target}:{cmd}")
        return False

    try:
        port.write((f"{target}:{cmd}\n").encode("utf-8"))
    except (serial.SerialException, OSError) as e:
        _handle_bridge_lost(e)
        return False

    _last_command[key] = now
    print(f"[CMD] Đã gửi: {target}:{cmd}")
    return True


def ack_fire_gas():
    global _manual_ack_ts
    _manual_ack_ts = time.time()
    print("[INFO] Đã ghi nhận người dùng xác nhận xử lý cảnh báo gas/cháy.")


# ==========================================
# NGỮ CẢNH HIỆN DIỆN / NHIỆT ĐỘ
# ==========================================
def _remember_kitchen_temp(temp):
    if temp is None:
        return
    now = time.time()
    _kit_temp_history.append((now, temp))
    while _kit_temp_history and now - _kit_temp_history[0][0] > TEMP_RISE_WINDOW_SEC:
        _kit_temp_history.pop(0)


def _kitchen_temp_delta():
    if len(_kit_temp_history) < 2:
        return 0.0
    return _kit_temp_history[-1][1] - _kit_temp_history[0][1]

def _is_night():
    hour = datetime.now().hour
    return hour >= NIGHT_START_HOUR or hour < NIGHT_END_HOUR

## Cập nhật thời điểm cuối cùng có người hiện diện trong nhà (motion PIR hoặc YOLO)
def _update_presence():
    global _last_presence_ts
    lr = living_room.state
    yolo_people = int(lr.get("yolo_people_count") or 0)
    present = (
        bool(state["bed_motion"]) or
        bool(lr.get("motion")) or
        yolo_people > 0
    )
    if present:
        _last_presence_ts = time.time()
    return present


def _is_unoccupied():
    _update_presence()
    return time.time() - _last_presence_ts >= UNOCCUPIED_AFTER_SEC


def _danger_detected():
    return bool(_danger_reasons())


# Hàm trả về danh sách lý do cảnh báo gas/cháy, để app hiển thị cho người dùng
def _danger_reasons():
    gas_threshold = state.get("kit_gas_threshold")
    gas_raw = state["kit_gas_raw"]
    temp = state["kit_temp"]
    temp_delta = _kitchen_temp_delta()

    reasons = []

    # Báo gas chỉ so gas_raw với ngưỡng node kitchen đang dùng.
    # Chưa biết ngưỡng (node chưa báo) thì không so sánh với None.
    if gas_raw is not None and gas_threshold is not None and gas_raw >= gas_threshold:
        reasons.append("gas")

    if temp is not None and temp >= TEMP_FIRE_THRESHOLD:
        reasons.append("high_temp")
    if temp_delta >= TEMP_RISE_THRESHOLD:
        reasons.append("temp_rise")
    return reasons


def _alarm_details():
    return {
        "kit_temp": state["kit_temp"],
        "kit_humid": state["kit_humid"],
        "kit_gas_raw": state["kit_gas_raw"],
        "kit_gas_voltage": state["kit_gas_voltage"],
        "kit_gas_threshold": state["kit_gas_threshold"],
        "delta_t_10s": round(_kitchen_temp_delta(), 2),
        "reason": state["fire_gas_reason"],
        "bed_motion": state["bed_motion"],
        "living_motion": bool(living_room.state.get("motion")),
        "yolo_people_count": int(living_room.state.get("yolo_people_count") or 0),
    }


def _set_fire_gas_alarm(context: str, level: str, unlock: bool = False):
    global _unlock_sent, _last_fb_alarm
    state["fire_gas_context"] = context
    state["fire_gas_level"] = level
    state["fire_gas_reason"] = ",".join(_danger_reasons())

    _publish_gas_alert(True)

    # Firebase: chỉ ghi khi đổi (context, level), không ghi lặp mỗi giây
    if (context, level) != _last_fb_alarm:
        _last_fb_alarm = (context, level)
        _notify_fire_gas_firebase(context, level)

    print(f"[FIRE/GAS] {state['fire_gas_reason']} -> {context}/{level}: gửi lệnh về các node")
    send_command("kitchen", "ALERT_ON")
    # Còi phòng ngủ luôn kêu mạnh nhất khi có báo động, bất kể mức MED/MAX/RED
    send_command("bedroom", "ALARM_MAX")

    # Phòng khách: bật đèn/quạt (không dùng còi)
    living_room.trigger_fire_gas_alarm(level, context, _alarm_details())

    if unlock and not _unlock_sent:
        # auto_relock=False: giữ cửa mở làm lối thoát hiểm, không tự khoá lại sau vài giây
        threading.Thread(
            target=living_room.unlock_door,
            kwargs={"source": f"fire_gas_{context}", "auto_relock": False},
            daemon=True,
        ).start()
        _unlock_sent = True


def _send_alarm_off_repeated():
    """Gửi lệnh tắt còi: lần đầu ngay lập tức, các lần sau trong thread riêng
    (ESP-NOW không có xác nhận nên gửi lặp để tránh mất gói, còi không kêu mãi)."""
    send_command("kitchen", "ALERT_OFF", force=True)
    send_command("bedroom", "ALARM_OFF", force=True)

    def _worker():
        for _ in range(ALARM_OFF_REPEAT - 1):
            time.sleep(ALARM_OFF_REPEAT_GAP_SEC)
            send_command("kitchen", "ALERT_OFF", force=True)
            send_command("bedroom", "ALARM_OFF", force=True)

    if ALARM_OFF_REPEAT > 1:
        threading.Thread(target=_worker, daemon=True).start()


def _clear_fire_gas_alarm():
    global _danger_started_ts, _day_escalated, _unlock_sent, _last_fb_alarm, _safe_since
    if state["fire_gas_level"] != "CLEAR":
        _send_alarm_off_repeated()
        living_room.clear_fire_gas_alarm()
        print("[INFO] Cảnh báo gas/cháy đã về CLEAR.")

        fb.add_event("fire_gas_clear", {"previous_level": state["fire_gas_level"]})
        fb.add_notification(
            "Đã hết cảnh báo gas/cháy",
            "Trạng thái trở về bình thường.",
            level="info",
            event_type="fire_gas_clear",
        )

        # Hết báo động thì khoá cửa lại (nếu lúc báo động đã mở cửa thoát hiểm
        # và cửa đang đóng hẳn; nếu cửa còn mở thì để nguyên, người dùng tự khoá qua app)
        if _unlock_sent and not living_room.state["door_open"]:
            threading.Thread(target=living_room.lock_door, daemon=True).start()

    _publish_gas_alert(False)

    state["fire_gas_context"] = "clear"
    state["fire_gas_level"] = "CLEAR"
    state["fire_gas_reason"] = "none"
    _danger_started_ts = None
    _safe_since = None
    _day_escalated = False
    _unlock_sent = False
    _last_fb_alarm = None


def evaluate_fire_gas_context():
    global _danger_started_ts, _day_escalated, _safe_since

    # Chưa có bridge thì không có dữ liệu bếp/phòng ngủ để đánh giá. Nếu đang KHÔNG báo động
    # thì bỏ qua (tránh báo "gas an toàn" giả lên app). Nếu ĐANG báo động thì vẫn đánh giá tiếp
    # để giữ nguyên báo động (an toàn hơn là tự tắt khi không còn dữ liệu).
    if not _bridge_online and state["fire_gas_level"] == "CLEAR":
        return

    now = time.time()
    if not _danger_detected():
        if state["fire_gas_level"] != "CLEAR":
            # Đang báo động: phải hết nguy hiểm liên tục CLEAR_HOLD_SEC mới tắt
            if _safe_since is None:
                _safe_since = now
            if now - _safe_since < CLEAR_HOLD_SEC:
                return
        _safe_since = None
        _clear_fire_gas_alarm()
        return

    _safe_since = None   # nguy hiểm quay lại: huỷ bộ đếm giữ báo động

    if _manual_ack_ts and _danger_started_ts and _manual_ack_ts >= _danger_started_ts:
        _set_fire_gas_alarm("acknowledged", "MED")
        return

    if _danger_started_ts is None:
        _danger_started_ts = now
        _day_escalated = False

    danger_age = now - _danger_started_ts

    if _is_unoccupied():
        _set_fire_gas_alarm("unoccupied", "RED", unlock=True)
        return

    if _is_night() and state["bed_motion"]:
        _set_fire_gas_alarm(
            "night_sleeping",
            "MAX",
            unlock=danger_age >= NIGHT_UNLOCK_AFTER_SEC,
        )
        return

    if danger_age >= DAY_ESCALATE_AFTER_SEC:
        _day_escalated = True

    if _day_escalated:
        _set_fire_gas_alarm("day_occupied_escalated", "MAX", unlock=True)
    else:
        _set_fire_gas_alarm("day_occupied", "MED")


# ==========================================
# PARSE DỮ LIỆU JSON TỪ BRIDGE
# ==========================================
def parse_data(line: str) -> bool:
    line = line.strip()
    if not line.startswith("{"):
        return False

    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        print(f"[WARN] JSON lỗi: {line}")
        return False

    if "error" in data:
        print(f"[ERROR] Bridge: {data['error']}")
        return False

    if "bridge_mac" in data:
        print(f"[INFO] Bridge MAC: {data['bridge_mac']}")
        return False

    node = data.get("node")
    ts = datetime.now().strftime("%H:%M:%S")

    if node == "kitchen":
        kit_temp = data.get("temp")
        kit_humid = data.get("humid")
        kit_gas_raw = data.get("gas_raw")
        kit_gas_voltage = data.get("gas_voltage", data.get("gas"))   # tuỳ chọn, chỉ để hiển thị

        # Chỉ gas_raw là bắt buộc: báo gas chỉ dựa vào nó.
        # DHT (temp/humid) lỗi thì KHÔNG được làm mất cảnh báo gas.
        if kit_gas_raw is None:
            print(f"[WARN] Kitchen thiếu gas_raw: {data}")
            return False

        if kit_temp is None or kit_humid is None:
            print("[WARN] Kitchen DHT đọc lỗi tạm thời, giữ nguyên nhiệt độ/độ ẩm trước đó.")
        else:
            state["kit_temp"] = round(float(kit_temp), 1)
            state["kit_humid"] = round(float(kit_humid), 1)
            _remember_kitchen_temp(state["kit_temp"])

        state["kit_gas_raw"] = int(kit_gas_raw)
        state["kit_gas_voltage"] = round(float(kit_gas_voltage), 3) if kit_gas_voltage is not None else None
        state["kit_fan"] = bool(data.get("fan", False))
        state["kit_led"] = bool(data.get("led", False))
        state["kit_alarm"] = bool(data.get("alarm", False))

        # Ngưỡng là field tuỳ chọn: firmware cũ chưa gửi thì giữ giá trị trước đó, không crash
        th_raw = data.get("gas_threshold")
        if th_raw is not None:
            try:
                state["kit_gas_threshold"] = int(th_raw)
                _publish_gas_threshold_state_to_app()   # app thấy đúng giá trị node đang dùng
            except (TypeError, ValueError):
                print(f"[WARN] Kitchen gas_threshold không hợp lệ: {th_raw!r}")

        _update_presence()

        print(f"[{ts}] KITCHEN | "
              f"Nhiệt độ: {state['kit_temp']}°C  "
              f"Độ ẩm: {state['kit_humid']}%  "
              f"Gas: raw={state['kit_gas_raw']} / {state['kit_gas_voltage']}V  "
              f"Ngưỡng: {state['kit_gas_threshold']}")

        safe_append_log({"ts": ts, "node": "kitchen", **{
            "temp": state["kit_temp"],
            "humid": state["kit_humid"],
            "gas_raw": state["kit_gas_raw"],
            "gas_voltage": state["kit_gas_voltage"],
            "gas_threshold": state["kit_gas_threshold"],
        }})
        evaluate_fire_gas_context()
        return True

    if node == "bedroom":
        bed_temp = data.get("temp")
        bed_humid = data.get("humid")
        bed_light = data.get("light")

        if bed_light is None:
            print(f"[WARN] Bedroom thiếu field ánh sáng: {data}")
            return False

        if bed_temp is None or bed_humid is None:
            print("[WARN] Bedroom DHT22 đọc lỗi tạm thời, giữ nguyên nhiệt độ/độ ẩm trước đó.")
        else:
            state["bed_temp"] = round(float(bed_temp), 1)
            state["bed_humid"] = round(float(bed_humid), 1)

        state["bed_light"] = round(float(bed_light), 1)
        state["bed_motion"] = bool(data.get("motion", False))
        state["bed_fan"] = bool(data.get("fan", False))
        state["bed_led"] = bool(data.get("led", False))
        state["bed_alarm_level"] = int(data.get("alarm_level", 0))

        _update_presence()

        print(f"[{ts}] BEDROOM | "
              f"Nhiệt độ: {state['bed_temp']}°C  "
              f"Độ ẩm: {state['bed_humid']}%  "
              f"Ánh sáng: {state['bed_light']} lux  "
              f"PIR: {'Có' if state['bed_motion'] else 'Không'}")

        safe_append_log({"ts": ts, "node": "bedroom", **{
            "temp": state["bed_temp"],
            "humid": state["bed_humid"],
            "light": state["bed_light"],
            "motion": state["bed_motion"],
        }})
        evaluate_fire_gas_context()
        return True

    return False


def cleanup():
    global ser, _log_fh
    try:
        _clear_fire_gas_alarm()
    except Exception:
        pass
    if ser is not None and ser.is_open:
        try:
            ser.close()
            print("[INFO] Đã đóng cổng serial.")
        except Exception:
            pass
    if _log_fh is not None:
        try:
            _log_fh.flush()
            os.fsync(_log_fh.fileno())
            _log_fh.close()
            print("[INFO] Đã đóng file log an toàn.")
        except Exception:
            pass
    try:
        living_room.stop()
        print("[INFO] Đã dọn dẹp phòng khách (GPIO, MQTT).")
    except Exception:
        pass
    try:
        fb.stop()       # đánh dấu gateway offline và xả hàng đợi ghi Firestore
    except Exception:
        pass
    cleanup_mqtt()


def handle_signal(signum, frame):
    global _shutdown
    if _shutdown:
        return
    _shutdown = True
    print(f"\n[INFO] Nhận tín hiệu dừng ({signum}), đang dọn dẹp...")
    cleanup()
    sys.exit(0)


def main():
    global _living_room_thread

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    living_room.set_event_callback(_on_living_room_event)
    setup_mqtt()
    fb.start()   # lỗi (thiếu thư viện / khoá / mạng) chỉ cảnh báo, gateway vẫn chạy

    _living_room_thread = threading.Thread(target=living_room.run, daemon=True)
    _living_room_thread.start()

    # Bridge có thể chưa cắm: thử mở 1 lần. Lỗi thì gateway VẪN CHẠY (phòng khách, MQTT, app)
    # và cứ BRIDGE_RETRY_SEC giây tự thử lại, nên cắm bridge vào sau cũng được.
    print(f"Đang kết nối bridge {PORT} @ {BAUD_RATE} baud ...")
    if not _open_serial():
        print("[INFO] Gateway vẫn chạy không có bridge, sẽ tự thử lại mỗi "
              f"{BRIDGE_RETRY_SEC:.0f} giây.")
    last_bridge_try = time.time()
    print("Gateway sẵn sàng.\n")

    last_house_print = 0.0
    last_context_eval = 0.0
    last_app_publish = 0.0
    last_fb_state = 0.0

    try:
        while True:
            now = time.time()

            # ---- Bridge: đọc dữ liệu, hoặc tự thử kết nối lại nếu chưa có / bị rút ----
            port = ser
            if port is None or not port.is_open:
                if now - last_bridge_try >= BRIDGE_RETRY_SEC:
                    last_bridge_try = now
                    _open_serial()
            else:
                raw = None
                try:
                    if port.in_waiting:
                        raw = port.readline().decode("utf-8", errors="replace")
                except (serial.SerialException, OSError) as e:
                    _handle_bridge_lost(e)
                if raw:
                    parse_data(raw)

            now = time.time()   # _open_serial() có thể mất ~2 s
            if now - last_context_eval >= 1.0:
                last_context_eval = now
                evaluate_fire_gas_context()
                _sync_device_states_from_nodes()   # đèn/quạt thật -> app (chỉ publish khi đổi)

            if now - last_app_publish >= APP_PUBLISH_INTERVAL_SEC:
                last_app_publish = now
                publish_sensors_to_app()

            if now - last_fb_state >= FIREBASE_STATE_INTERVAL_SEC:
                last_fb_state = now
                fb.publish_state(_firebase_state_snapshot())

            if now - last_house_print >= 5.0:
                last_house_print = now
                lr = living_room.state
                ts = datetime.now().strftime("%H:%M:%S")
                bridge_txt = "OK" if _bridge_online else "MẤT"
                print(f"[{ts}] =========== TOÀN NHÀ =========| "
                      f"Bridge: {bridge_txt}  "
                      f"Kitchen: {state['kit_temp']}°C/{state['kit_humid']}%/"
                      f"gas={state['kit_gas_raw']}  "
                      f"Ngưỡng: {state.get('kit_gas_threshold')}  "
                      f"Bedroom: {state['bed_temp']}°C/{state['bed_humid']}%  "
                      f"Living room: {lr['temp']}°C/{lr['humid']}%  "
                      f"Người: {'Có' if _update_presence() else 'Không'}  "
                      f"Cửa: {'Mở' if lr['door_open'] else 'Đóng'} / "
                      f"Khoá: {'Có' if lr['door_locked'] else 'Không'}  "
                      f"Gas/Fire: {state['fire_gas_context']}/{state['fire_gas_level']}\n")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Đã dừng (Ctrl+C).")
    finally:
        cleanup()


if __name__ == "__main__":
    main()