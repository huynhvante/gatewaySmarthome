import os
import time
import json
import threading
from datetime import datetime

import RPi.GPIO as GPIO
import smbus2
import board
import adafruit_dht

# ==========================================
# CẤU HÌNH CHUNG
# ==========================================
PIR_PIN      = 27      
REED_PIN     = 17      

SERVO_PIN    = 18     
BUZZER_PIN   = 25     

FAN_INA_PIN  = 23     
FAN_INB_PIN  = 24
FAN_PWM_FREQ = 1000
FAN_SPEED    = 80      

LED_PIN      = 22     
DHT_PIN      = board.D4           
I2C_BUS      = 1
BH1750_ADDR  = 0x23

# ==========================================
# NGƯỠNG TỰ ĐỘNG (giống logic bên bedroom)
# ==========================================
LUX_THRESHOLD   = 50.0    
MOTION_HOLD_SEC = 60     

UNLOCK_GRACE_SEC    = 15   
AUTO_LOCK_AFTER_SEC = 6    

MANUAL_OVERRIDE_SEC = 5 * 60   


# ==========================================
# TRẠNG THÁI TOÀN CỤC
# ==========================================
state = {
    "motion": False,
    "last_motion_ts": 0.0,
    "temp": None,
    "humid": None,
    "lux": None,
    "fan": False,
    "led": False,
    "door_locked": True,
    "door_open": False,
    "yolo_people_count": 0,
    "last_yolo_ts": 0.0,
    "fire_gas_alarm": False,
    "fire_gas_level": "CLEAR",
}

_last_authorized_unlock_ts = 0.0
_dht = None
i2c_bus = None
_servo_pwm = None
_fan_pwm = None
_stop_flag = False
_last_lux_warn_ts = 0.0
_event_callback = None

# [MỚI] Mốc thời gian (epoch) mà trước đó auto-logic KHÔNG được ghi đè trạng thái
# quạt/đèn, vì người dùng vừa điều khiển tay qua app.
_fan_manual_until = 0.0
_led_manual_until = 0.0


def set_event_callback(callback):
    """Gateway đăng ký callback để nhận sự kiện phòng khách và publish MQTT tập trung."""
    global _event_callback
    _event_callback = callback


def _emit_event(event_type: str, payload: dict):
    if _event_callback is None:
        return
    try:
        _event_callback(event_type, payload)
    except Exception as e:
        print(f"[WARN] Callback gateway lỗi: {e}")


# ==========================================
# GPIO / CẢM BIẾN - KHỞI TẠO
# ==========================================
def setup_gpio():
    global _servo_pwm, _fan_pwm, _dht, i2c_bus

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(PIR_PIN, GPIO.IN)
    GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # đóng cửa = mạch kín = LOW
    GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(FAN_INA_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(FAN_INB_PIN, GPIO.OUT, initial=GPIO.LOW)
    _fan_pwm = GPIO.PWM(FAN_INB_PIN, FAN_PWM_FREQ)
    _fan_pwm.start(0)

    GPIO.setup(SERVO_PIN, GPIO.OUT)
    _servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz cho servo
    _servo_pwm.start(0)

    _dht = adafruit_dht.DHT22(getattr(board, f"D{DHT_PIN}"), use_pulseio=False)

    try:
        i2c_bus = smbus2.SMBus(I2C_BUS)
    except PermissionError:
        i2c_bus = None
        print(
            f"[WARN] Không có quyền mở /dev/i2c-{I2C_BUS}. "
            f"User hiện tại: {os.getenv('USER', 'unknown')}. "
            "BH1750/lux sẽ tạm bị bỏ qua, các chức năng gas/cửa/còi vẫn chạy."
        )
    except FileNotFoundError:
        i2c_bus = None
        print(f"[WARN] Không tìm thấy /dev/i2c-{I2C_BUS}. Kiểm tra đã bật I2C trên máy chưa.")

    GPIO.add_event_detect(PIR_PIN, GPIO.RISING, callback=_on_motion, bouncetime=300)
    GPIO.add_event_detect(REED_PIN, GPIO.BOTH, callback=_on_door_state_change, bouncetime=200)


# ==========================================
# QUẠT (bật theo ngưỡng nhiệt độ, giống logic bedroom)
# ==========================================
def fan_on():
    GPIO.output(FAN_INA_PIN, GPIO.LOW)
    _fan_pwm.ChangeDutyCycle(FAN_SPEED)
    state["fan"] = True


def fan_off():
    GPIO.output(FAN_INA_PIN, GPIO.LOW)
    _fan_pwm.ChangeDutyCycle(0)
    state["fan"] = False


# [MỚI] Điều khiển quạt thủ công (từ app Android qua gateway).
# Khác với fan_on()/fan_off(): hàm này còn tạm khoá auto-logic (PIR) trong
# MANUAL_OVERRIDE_SEC giây để lệnh của người dùng không bị vòng lặp run() ghi đè ngay.
def set_fan(on: bool, source: str = "app"):
    global _fan_manual_until
    if on:
        fan_on()
    else:
        fan_off()
    _fan_manual_until = time.time() + MANUAL_OVERRIDE_SEC
    print(f"[LIVING ROOM] Quạt điều khiển thủ công bởi {source}: {'ON' if on else 'OFF'}")


# ==========================================
# ĐÈN (bật khi tối + có người)
# ==========================================
def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)
    state["led"] = True


def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)
    state["led"] = False


# NHẬN LỆNH ĐIỀU KHIỂN TỪ APP android
def set_led(on: bool, source: str = "app"):
    global _led_manual_until
    if on:
        led_on()
    else:
        led_off()
    _led_manual_until = time.time() + MANUAL_OVERRIDE_SEC
    print(f"[LIVING ROOM] Đèn điều khiển thủ công bởi {source}: {'ON' if on else 'OFF'}")


# ==========================================
# SERVO KHOÁ CỬA
# ==========================================
def _set_servo_angle(angle: float):
    duty = 2.5 + (angle / 180.0) * 10.0   # 0°->2.5%  180°->12.5%  (chỉnh theo servo thực tế)
    _servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)
    _servo_pwm.ChangeDutyCycle(0)   # nhả xung để servo không rung/nóng khi giữ nguyên vị trí


def lock_door():
    _set_servo_angle(0)     # góc khoá - chỉnh theo cơ cấu khoá thực tế
    state["door_locked"] = True
    _emit_door_status()


def unlock_door(source: str = "app"):
    global _last_authorized_unlock_ts
    _set_servo_angle(90)    # góc mở khoá - chỉnh theo cơ cấu khoá thực tế
    state["door_locked"] = False
    _last_authorized_unlock_ts = time.time()
    print(f"[DOOR] Mở khoá bởi: {source}")
    _emit_door_status()

    def _auto_relock():
        time.sleep(AUTO_LOCK_AFTER_SEC)
        if not state["door_locked"]:
            lock_door()

    threading.Thread(target=_auto_relock, daemon=True).start()


# ==========================================
# CẢM BIẾN TỪ CỬA + BÁO ĐỘNG CHỐNG ĐỘT NHẬP
# ==========================================
def _apply_door_state(is_open: bool):
    """Chỉ đọc và ghi nhận trạng thái cửa — chưa xử lý cảnh báo đột nhập."""
    if state["door_open"] == is_open:
        return  # không đổi, không cần xử lý lại

    state["door_open"] = is_open
    ts = datetime.now().strftime("%H:%M:%S")

    if is_open:
        print(f"[{ts}] [DOOR] Cửa đang mở.")
    else:
        print(f"[{ts}] [DOOR] Cửa đã đóng.")

    _emit_door_status()


def _on_door_state_change(channel):
    is_open = GPIO.input(REED_PIN) == GPIO.HIGH   # HIGH = mạch hở = cửa đang mở
    _apply_door_state(is_open)


# ==========================================
# PIR - PHÁT HIỆN CHUYỂN ĐỘNG
# ==========================================
def _on_motion(channel):
    state["motion"] = True
    state["last_motion_ts"] = time.time()


def _is_occupied() -> bool:
    return (time.time() - state["last_motion_ts"]) < MOTION_HOLD_SEC


def _emit_door_status():
    _emit_event("door_status", {
        "locked": state["door_locked"],
        "open": state["door_open"],
        "ts": datetime.now().isoformat(),
    })


def _emit_alarm(alarm_type: str):
    _emit_event("alarm", {
        "type": alarm_type,
        "ts": datetime.now().isoformat(),
    })


def set_yolo_people_count(count: int):
    """Cho tiến trình YOLO cập nhật số người thấy được trong phòng khách."""
    state["yolo_people_count"] = max(0, int(count))
    state["last_yolo_ts"] = time.time()


def trigger_fire_gas_alarm(level: str, context: str, details: dict | None = None):
    state["fire_gas_alarm"] = True
    state["fire_gas_level"] = level

    fan_on()
    led_on()
    if level in ("MAX", "RED"):
        GPIO.output(BUZZER_PIN, GPIO.HIGH)

    payload = {
        "type": "fire_gas",
        "level": level,
        "context": context,
        "details": details or {},
        "ts": datetime.now().isoformat(),
    }
    _emit_event("alarm", payload)
    _emit_event("fire_gas", payload)


def clear_fire_gas_alarm():
    state["fire_gas_alarm"] = False
    state["fire_gas_level"] = "CLEAR"
    if not state["door_open"]:
        GPIO.output(BUZZER_PIN, GPIO.LOW)
    _emit_event("fire_gas", {
        "type": "fire_gas",
        "level": "CLEAR",
        "ts": datetime.now().isoformat(),
    })


# ==========================================
# ĐỌC CẢM BIẾN ÁNH SÁNG (BH1750)
# ==========================================
def _read_lux():
    global _last_lux_warn_ts
    if i2c_bus is None:
        return None

    for attempt in range(3):
        try:
            data = i2c_bus.read_i2c_block_data(BH1750_ADDR, 0x10, 2)
            lux = (data[0] << 8 | data[1]) / 1.2   # công thức chuyển đổi theo datasheet BH1750
            return round(lux, 1)
        except OSError as e:
            if getattr(e, "errno", None) == 11 and attempt < 2:
                time.sleep(0.05)
                continue
            now = time.time()
            if now - _last_lux_warn_ts >= 30:
                _last_lux_warn_ts = now
                print(f"[WARN] Đọc BH1750 lỗi: {e}")
            return None
        except Exception as e:
            now = time.time()
            if now - _last_lux_warn_ts >= 30:
                _last_lux_warn_ts = now
                print(f"[WARN] Đọc BH1750 lỗi: {e}")
            return None


# ==========================================
# ĐỌC NHIỆT ĐỘ / ĐỘ ẨM (DHT22)
# ==========================================
def _read_dht():
    try:
        return round(_dht.temperature, 1), round(_dht.humidity, 1)
    except RuntimeError:
        return None, None  

# ==========================================
# VÒNG LẶP CHÍNH CỦA PHÒNG KHÁCH
# ==========================================
def run():
    """Gọi hàm này trong 1 thread riêng từ gateway.py (xem main())."""
    setup_gpio()
    _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)   # đọc trạng thái cửa thật lúc khởi động
    lock_door()

    print("[LIVING ROOM] Sẵn sàng.")
    last_sensor_read = 0.0

    while not _stop_flag:
        now = time.time()
        _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)   # poll dự phòng, không phụ thuộc hoàn toàn vào interrupt

        if now - last_sensor_read >= 2.0:
            last_sensor_read = now

            temp, humid = _read_dht()
            if temp is not None:
                state["temp"], state["humid"] = temp, humid

            lux = _read_lux()
            if lux is not None:
                state["lux"] = lux

            occupied = _is_occupied()
            state["motion"] = occupied

            # Quạt: theo hiện diện, giữ 60 giây sau lần PIR cuối.
            # [SỬA] Bỏ qua nếu đang trong thời gian giữ trạng thái điều khiển tay từ app.
            if time.time() >= _fan_manual_until:
                if occupied and not state["fan"]:
                    fan_on()
                elif not occupied and state["fan"]:
                    fan_off()

            # Đèn: tối + có người
            # [SỬA] Bỏ qua nếu đang trong thời gian giữ trạng thái điều khiển tay từ app.
            if state["lux"] is not None and time.time() >= _led_manual_until:
                should_light = state["lux"] < LUX_THRESHOLD and occupied
                if should_light and not state["led"]:
                    led_on()
                elif not should_light and state["led"]:
                    led_off()

            ts = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts}] LIVING ROOM | "
                  f"Nhiệt độ: {state['temp']}°C  Độ ẩm: {state['humid']}%  "
                  f"Ánh sáng: {state['lux']} lux  "
                  f"Người: {'Có' if occupied else 'Không'}  "
                  f"Quạt: {'ON' if state['fan'] else 'OFF'}  "
                  f"Đèn: {'ON' if state['led'] else 'OFF'}")
            _emit_event("sensor_reading", {          # ← thêm đoạn này
                "ts": ts,
                "node": "living_room",
                "temp": state["temp"],
                "humid": state["humid"],
                "lux": state["lux"],
                "motion": occupied,
                "fan": state["fan"],
                "led": state["led"],
                "door_open": state["door_open"],
                "door_locked": state["door_locked"],
            })
        time.sleep(0.2)


def stop():
    global _stop_flag
    _stop_flag = True
    try:
        fan_off()
        led_off()
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        _servo_pwm.stop()
        _fan_pwm.stop()
        GPIO.cleanup()
    except Exception:
        pass