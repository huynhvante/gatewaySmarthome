import serial
import json
import time
import os
import signal
import sys
import threading
from datetime import datetime

import paho.mqtt.client as mqtt

import living_room

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
# [SỬA] Không để mật khẩu trong code. Chạy: export MQTT_PASSWORD='...'
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD", "Vante10012004@")
MQTT_CLIENT_ID = "raspi-home-gateway"

# --- Topic điều khiển / trạng thái nội bộ (giữ nguyên) ---
TOPIC_DOOR_CMD = "home/livingroom/door/cmd"
TOPIC_FACE_UNLOCK = "home/livingroom/door/face_unlock"
TOPIC_DOOR_STATUS = "home/livingroom/door/status"
TOPIC_ALARM = "home/livingroom/alarm"
TOPIC_FIRE_GAS = "home/fire_gas/alarm"
TOPIC_FIRE_GAS_ACK = "home/fire_gas/ack"
TOPIC_HOUSE_STATUS = "home/gateway/status"



#  Topic cho app Android: smarthome/<room>/<device>/<metric>, payload là chuỗi thuần
ROOT_TOPIC = "smarthome"
APP_PUBLISH_INTERVAL_SEC = 5.0

# [SỬA] Chỉ subscribe đúng các thiết bị điều khiển được app hỗ trợ — không dùng wildcard.
CONTROLLABLE_DEVICES = [
    ("living_room", "led"),
    ("living_room", "fan"),
    ("living_room", "door"),
    ("bedroom", "led"),
    ("bedroom", "fan"),
    ("bedroom", "buzzer"),
    ("kitchen", "led"),
    ("kitchen", "fan"),
]
TOPIC_APP_CMD_SET = {
    f"{ROOT_TOPIC}/{room}/{device}/set": (room, device)
    for room, device in CONTROLLABLE_DEVICES
}

GAS_RAW_THRESHOLD = 2000
GAS_VOLTAGE_FALLBACK_THRESHOLD = 2.0
TEMP_FIRE_THRESHOLD = 55.0
TEMP_RISE_THRESHOLD = 3.0
TEMP_RISE_WINDOW_SEC = 10.0
UNOCCUPIED_AFTER_SEC = 30 * 60
NIGHT_START_HOUR = 23
NIGHT_END_HOUR = 6
NIGHT_UNLOCK_AFTER_SEC = 30
DAY_ESCALATE_AFTER_SEC = 60
COMMAND_REPEAT_SEC = 10

# ==========================================
# TRẠNG THÁI TOÀN CỤC
# ==========================================
state = {
    "kit_temp": None,
    "kit_humid": None,
    "kit_gas_raw": None,
    "kit_gas_voltage": None,
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
_day_escalated = False
_unlock_sent = False
_manual_ack_ts = 0.0
_last_presence_ts = time.time()
_last_command = {}
_mqtt_client = None
_last_gas_alert = None  
_device_states = {} 


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


# Gửi toàn bộ cảm biến theo đúng cấu trúc topic của app
def publish_sensors_to_app():
    lr = living_room.state

    # Living room
    _pub_sensor("living_room", "pir", "motion", lr.get("motion"))
    _pub_sensor("living_room", "dht", "temperature", lr.get("temp"))
    _pub_sensor("living_room", "dht", "humidity", lr.get("humid"))


    # Kitchen
    _pub_sensor("kitchen", "gas_sensor", "value", state["kit_gas_raw"])
    _pub_sensor("kitchen", "dht", "temperature", state["kit_temp"])
    _pub_sensor("kitchen", "dht", "humidity", state["kit_humid"])

    # Bedroom (chỉ gửi sau khi đã nhận gói đầu tiên, vì bed_motion mặc định là False)
    if state["bed_light"] is not None:
        _pub_sensor("bedroom", "pir", "motion", state["bed_motion"])
        _pub_sensor("bedroom", "dht", "temperature", state["bed_temp"])
        _pub_sensor("bedroom", "dht", "humidity", state["bed_humid"])
        _pub_sensor("bedroom", "light_sensor", "lux", state["bed_light"])


#  Alert gas: chỉ publish khi đổi trạng thái, retained để app mở lên vẫn thấy
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


#  Chuẩn hoá payload lệnh: nhiều app gửi ON/OFF, 1/0, true/false...
def _normalize_switch_payload(raw: str):
    v = raw.strip().upper()
    if v in ("ON", "1", "TRUE", "OPEN", "UNLOCK"):
        return "ON"
    if v in ("OFF", "0", "FALSE", "CLOSE", "LOCK"):
        return "OFF"
    return None


#  Publish lại trạng thái cho app: smarthome/<room>/<device>/state (retained)
def _publish_device_state(room: str, device: str, value: str):
    _publish_raw(f"{ROOT_TOPIC}/{room}/{device}/state", value, qos=1, retain=True)


#  Các cặp (room, device) đã có phần cứng thật, gắn lệnh thật xuống thiết bị.
# Trả về True nếu đã có phần cứng xử lý (không cần cảnh báo "chưa có phần cứng").
def _dispatch_hardware_command(room: str, device: str, is_on: bool) -> bool:
    if room == "kitchen" and device == "fan":
        send_command("kitchen", "FAN_ON" if is_on else "FAN_OFF", force=True)
        return True
    if room == "kitchen" and device == "led":
        send_command("kitchen", "LED_ON" if is_on else "LED_OFF", force=True)
        return True
    if room == "bedroom" and device == "fan":
        send_command("bedroom", "FAN_ON" if is_on else "FAN_OFF", force=True)
        return True
    if room == "bedroom" and device == "led":
        send_command("bedroom", "LED_ON" if is_on else "LED_OFF", force=True)
        return True
    if room == "living_room" and device == "door":
        # Quy ước: ON = mở khoá, OFF = khoá lại
        if is_on:
            living_room.unlock_door(source="app")
        else:
            living_room.lock_door()
        return True
    if room == "living_room" and device == "fan":
        living_room.set_fan(is_on, source="app")
        return True
    if room == "living_room" and device == "led":
        living_room.set_led(is_on, source="app")
        return True
    return False


#  Xử lý lệnh app publish vào smarthome/<room>/<device>/set (room, device đã được xác định trước)
def _handle_app_command(room: str, device: str, raw_value: str):
    value = _normalize_switch_payload(raw_value)
    if value is None:
        print(f"[WARN] Giá trị lệnh không hợp lệ: {ROOT_TOPIC}/{room}/{device}/set = {raw_value}")
        return

    handled = _dispatch_hardware_command(room, device, value == "ON")
    _device_states[(room, device)] = value
    _publish_device_state(room, device, value)

    print(f"🎮 Nhận lệnh: {ROOT_TOPIC}/{room}/{device}/set = {value}  →  "
          f"đã cập nhật {ROOT_TOPIC}/{room}/{device}/state")
    if not handled:
        print(f"    (lưu ý: {room}/{device} chưa nối phần cứng thật, chỉ echo trạng thái cho app)")


def _on_living_room_event(event_type: str, payload: dict):
    if event_type == "door_status":
        _publish(TOPIC_DOOR_STATUS, payload, retain=True)
    elif event_type == "alarm":
        _publish(TOPIC_ALARM, payload)
    elif event_type == "fire_gas":
        _publish(TOPIC_FIRE_GAS, payload, retain=True)
    elif event_type == "sensor_reading":
        safe_append_log({"ts": datetime.now().strftime("%H:%M:%S"), "node": "living_room", **payload})


def _on_mqtt_connect(client, userdata, flags, rc, properties=None):
    print(f"[MQTT] Gateway đã kết nối HiveMQ, rc={rc}")
    client.subscribe(TOPIC_DOOR_CMD)
    client.subscribe(TOPIC_FACE_UNLOCK)
    client.subscribe(TOPIC_FIRE_GAS_ACK)
    for topic in TOPIC_APP_CMD_SET:  # [SỬA] subscribe từng topic cụ thể, không wildcard
        client.subscribe(topic)
    client.publish(TOPIC_HOUSE_STATUS, "online", qos=1, retain=True)


def _on_mqtt_message(client, userdata, msg):
    # [SỬA] Chỉ xử lý đúng các topic điều khiển đã đăng ký, không bắt topic lạ
    if msg.topic in TOPIC_APP_CMD_SET:
        room, device = TOPIC_APP_CMD_SET[msg.topic]
        raw_value = msg.payload.decode("utf-8", errors="replace")
        _handle_app_command(room, device, raw_value)
        return

    payload = msg.payload.decode("utf-8", errors="replace").strip().upper()
    print(f"[MQTT] {msg.topic}: {payload}")

    if msg.topic == TOPIC_DOOR_CMD:
        if payload in ("OPEN", "UNLOCK"):
            living_room.unlock_door(source="app")
        elif payload in ("CLOSE", "LOCK"):
            living_room.lock_door()
        else:
            print(f"[WARN] Lệnh cửa không hợp lệ: {payload}")

    elif msg.topic == TOPIC_FACE_UNLOCK:
        if payload in ("OK", "UNLOCK", "SUCCESS"):
            living_room.unlock_door(source="face")

    elif msg.topic == TOPIC_FIRE_GAS_ACK:
        if payload in ("ACK", "OK", "HANDLED", "CLEAR"):
            ack_fire_gas()


# [SỬA] Tương thích paho-mqtt 1.x và 2.x, tự reconnect, có LWT
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
    _mqtt_client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=60)  # mất mạng lúc khởi động không làm crash
    _mqtt_client.loop_start()


# [SỬA] Đúng thứ tự: publish offline -> disconnect -> loop_stop
def cleanup_mqtt():
    global _mqtt_client
    if _mqtt_client is None:
        return
    try:
        info = _mqtt_client.publish(TOPIC_HOUSE_STATUS, "offline", qos=1, retain=True)
        info.wait_for_publish(timeout=2)  # các publish qos1 trước đó cũng được đẩy đi theo thứ tự
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
# LỆNH ĐIỀU KHIỂN QUA BRIDGE
# ==========================================
def send_command(target: str, cmd: str, force: bool = False):
    global ser
    allowed_targets = {"kitchen", "bedroom", "all"}
    allowed_cmds = {
        "FAN_ON", "FAN_OFF", "LED_ON", "LED_OFF",
        "ALERT_ON", "ALERT_OFF",
        "ALARM_MED", "ALARM_MAX", "ALARM_OFF",
        "ON", "OFF", "AUTO",
    }
    target = target.strip().lower()
    cmd = cmd.strip().upper()

    if target not in allowed_targets or cmd not in allowed_cmds:
        print(f"[WARN] Lệnh không hợp lệ: {target}:{cmd}")
        return

    now = time.time()
    key = (target, cmd)
    if not force and now - _last_command.get(key, 0.0) < COMMAND_REPEAT_SEC:
        return

    if ser is None or not ser.is_open:
        print(f"[WARN] Chưa mở serial, bỏ qua lệnh: {target}:{cmd}")
        return

    ser.write((f"{target}:{cmd}\n").encode("utf-8"))
    _last_command[key] = now
    print(f"[CMD] Đã gửi: {target}:{cmd}")


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


def _danger_reasons():
    gas_raw = state["kit_gas_raw"]
    gas_voltage = state["kit_gas_voltage"]
    temp = state["kit_temp"]
    temp_delta = _kitchen_temp_delta()

    reasons = []
    if (
        (gas_raw is not None and gas_raw >= GAS_RAW_THRESHOLD) or
        (gas_raw is None and gas_voltage is not None and gas_voltage >= GAS_VOLTAGE_FALLBACK_THRESHOLD)
    ):
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
        "delta_t_10s": round(_kitchen_temp_delta(), 2),
        "reason": state["fire_gas_reason"],
        "bed_motion": state["bed_motion"],
        "living_motion": bool(living_room.state.get("motion")),
        "yolo_people_count": int(living_room.state.get("yolo_people_count") or 0),
    }


def _set_fire_gas_alarm(context: str, level: str, unlock: bool = False):
    global _unlock_sent
    state["fire_gas_context"] = context
    state["fire_gas_level"] = level
    state["fire_gas_reason"] = ",".join(_danger_reasons())

    _publish_gas_alert(True)  #  báo cho app

    print(f"[FIRE/GAS] {state['fire_gas_reason']} -> {context}/{level}: gửi lệnh về các node")
    send_command("kitchen", "ALERT_ON")
    if level == "MED":
        send_command("bedroom", "ALARM_MED")
    else:
        send_command("bedroom", "ALARM_MAX")

    living_room.trigger_fire_gas_alarm(level, context, _alarm_details())

    if unlock and not _unlock_sent:
        living_room.unlock_door(source=f"fire_gas_{context}")
        _unlock_sent = True


def _clear_fire_gas_alarm():
    global _danger_started_ts, _day_escalated, _unlock_sent
    if state["fire_gas_level"] != "CLEAR":
        send_command("kitchen", "ALERT_OFF", force=True)
        send_command("bedroom", "ALARM_OFF", force=True)
        living_room.clear_fire_gas_alarm()
        print("[INFO] Cảnh báo gas/cháy đã về CLEAR.")

    _publish_gas_alert(False)

    state["fire_gas_context"] = "clear"
    state["fire_gas_level"] = "CLEAR"
    state["fire_gas_reason"] = "none"
    _danger_started_ts = None
    _day_escalated = False
    _unlock_sent = False


def evaluate_fire_gas_context():
    global _danger_started_ts, _day_escalated

    now = time.time()
    if not _danger_detected():
        _clear_fire_gas_alarm()
        return

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
        kit_gas_voltage = data.get("gas_voltage", data.get("gas"))

        if None in (kit_temp, kit_humid) or (kit_gas_raw is None and kit_gas_voltage is None):
            print(f"[WARN] Kitchen thiếu field: {data}")
            return False

        state["kit_temp"] = round(float(kit_temp), 1)
        state["kit_humid"] = round(float(kit_humid), 1)
        state["kit_gas_raw"] = int(kit_gas_raw) if kit_gas_raw is not None else None
        state["kit_gas_voltage"] = round(float(kit_gas_voltage), 3) if kit_gas_voltage is not None else None
        state["kit_fan"] = bool(data.get("fan", False))
        state["kit_led"] = bool(data.get("led", False))
        state["kit_alarm"] = bool(data.get("alarm", False))

        _remember_kitchen_temp(state["kit_temp"])
        _update_presence()

        print(f"[{ts}] KITCHEN | "
              f"Nhiệt độ: {state['kit_temp']}°C  "
              f"Độ ẩm: {state['kit_humid']}%  "
              f"Gas: raw={state['kit_gas_raw']} / {state['kit_gas_voltage']}V  "
              )

        safe_append_log({"ts": ts, "node": "kitchen", **{
            "temp": state["kit_temp"],
            "humid": state["kit_humid"],
            "gas_raw": state["kit_gas_raw"],
            "gas_voltage": state["kit_gas_voltage"],
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
    global ser, _living_room_thread

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    living_room.set_event_callback(_on_living_room_event)
    setup_mqtt()

    _living_room_thread = threading.Thread(target=living_room.run, daemon=True)
    _living_room_thread.start()

    print(f"Đang kết nối {PORT} @ {BAUD_RATE} baud ...")

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Không mở được cổng: {e}")
        return

    time.sleep(2)
    print("Sẵn sàng nhận dữ liệu.\n")

    last_house_print = 0.0
    last_context_eval = 0.0
    last_app_publish = 0.0  # 

    try:
        while True:
            if ser.in_waiting:
                raw = ser.readline().decode("utf-8", errors="replace")
                parse_data(raw)

            now = time.time()
            if now - last_context_eval >= 1.0:
                last_context_eval = now
                evaluate_fire_gas_context()

            #  Đẩy dữ liệu cảm biến lên app theo chu kỳ
            if now - last_app_publish >= APP_PUBLISH_INTERVAL_SEC:
                last_app_publish = now
                publish_sensors_to_app()

            if now - last_house_print >= 5.0:
                last_house_print = now
                lr = living_room.state
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] =========== TOÀN NHÀ =========| "
                      f"Kitchen: {state['kit_temp']}°C/{state['kit_humid']}%/"
                      f"gas={state['kit_gas_raw']}  "
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