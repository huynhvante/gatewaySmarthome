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

SERVO_PIN    = 18      # SG90 - dây tín hiệu vào GPIO18 (chân vật lý 12)
BUZZER_PIN   = 25

FAN_INA_PIN  = 23
FAN_INB_PIN  = 24
FAN_PWM_FREQ = 1000
FAN_SPEED    = 80

LED_PIN      = 22
DHT_PIN      = board.D4           # truyền thẳng vào adafruit_dht.DHT22
I2C_BUS      = 1
BH1750_ADDR  = 0x23

# ==========================================
# CẤU HÌNH SERVO SG90 (đã test thực tế)
# ==========================================
SERVO_FREQ         = 50
SERVO_MIN_DUTY     = 2.5     # ~0.5 ms -> 0°
SERVO_MAX_DUTY     = 12.0    # ~2.4 ms -> 180°
SERVO_LOCK_ANGLE   = 0       # góc khoá
SERVO_UNLOCK_ANGLE = 90      # góc mở khoá
SERVO_MOVE_SEC     = 0.3     # giữ xung 0.3 s rồi nhả (test cho thấy không còn giật)

# ==========================================
# NGƯỠNG TỰ ĐỘNG
# ==========================================
LUX_THRESHOLD   = 50.0
MOTION_HOLD_SEC = 60

AUTO_LOCK_AFTER_SEC = 3      # mở khoá 3 giây rồi tự khoá lại (theo đề cương)
DOOR_OPEN_WAIT_MAX  = 60     # nếu cửa vẫn đang mở thì chờ tối đa 60 s rồi mới khoá

MANUAL_OVERRIDE_SEC = 5 * 60

# ==========================================
# NÚT NHẤN TÍCH HỢP LED
# SIG2: ngõ ra trạng thái nút (mặc định HIGH, nhấn = LOW)
# SIG1: ngõ vào điều khiển LED nút (mặc định LOW, HIGH = sáng)
# ==========================================
BTN_LED_SIG2 = 5      # nút vàng (đèn)
BTN_LED_SIG1 = 6
BTN_FAN_SIG2 = 13     # nút xanh (quạt)
BTN_FAN_SIG1 = 26
BTN_DEBOUNCE_SEC = 0.03
BTN_LONG_PRESS_SEC = 2.0
BTN_OVERRIDE_RELEASE_SEC = 30 * 60   # không có chuyển động bấy lâu -> về auto (0 = giữ mãi)

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

# Quạt ramp: tránh sụt áp khi bật quạt từ 0 -> 100% duty cycle
_fan_ramp_stop = threading.Event()
_fan_ramp_thread = None
FAN_RAMP_TIME = 1.5   
FAN_RAMP_STEPS = 20    

# Trước mốc này auto-logic KHÔNG được ghi đè quạt/đèn (người dùng vừa điều khiển tay qua app)
_fan_manual_until = 0.0
_led_manual_until = 0.0

# Nút nhấn quyết định thiết bị (ưu tiên hơn tự động)
_btn_override = {"led": False, "fan": False}
_last_btn_ts = 0.0
_btn_thread = None

# Khoá servo: tránh nhiều luồng cùng điều khiển servo một lúc
_servo_lock = threading.Lock()
# "Số thế hệ" của lần mở/khoá gần nhất: luồng auto-relock cũ tự huỷ nếu có lệnh mới hơn
_unlock_generation = 0


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
    _servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
    _servo_pwm.start(0)   # duty 0 = không phát xung, servo đứng yên

    GPIO.setup(BTN_LED_SIG2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_FAN_SIG2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_LED_SIG1, GPIO.OUT, initial=GPIO.HIGH)   # đèn đang tắt -> LED nút sáng
    GPIO.setup(BTN_FAN_SIG1, GPIO.OUT, initial=GPIO.HIGH)

    _dht = adafruit_dht.DHT22(DHT_PIN, use_pulseio=False)

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
# QUẠT
# ==========================================

# điều khiển mở quạt từ từ để tránh sụt áp (INA=LOW, INB=PWM)
def _fan_ramp(target):
    step_delay = FAN_RAMP_TIME / FAN_RAMP_STEPS
    for i in range(1, FAN_RAMP_STEPS + 1):
        if _fan_ramp_stop.is_set():
            return                      # bị ngắt (vd fan_off được gọi giữa chừng)
        _fan_pwm.ChangeDutyCycle(target * i / FAN_RAMP_STEPS)
        time.sleep(step_delay)


_fan_lock = threading.RLock() # tránh nhiều luồng cùng bật/tắt quạt một lúc

def fan_on():
    global _fan_ramp_thread
    with _fan_lock:
        if state["fan"]:
            return                      # đang chạy rồi, không ramp lại
        GPIO.output(FAN_INA_PIN, GPIO.LOW)
        _fan_ramp_stop.clear()
        _fan_ramp_thread = threading.Thread(target=_fan_ramp, args=(FAN_SPEED,), daemon=True)
        _fan_ramp_thread.start()
        state["fan"] = True


def fan_off():
    with _fan_lock:
        _fan_ramp_stop.set()            # dừng ramp đang chạy (nếu có)
        if _fan_ramp_thread is not None:
            _fan_ramp_thread.join()
        GPIO.output(FAN_INA_PIN, GPIO.LOW)
        _fan_pwm.ChangeDutyCycle(0)
        state["fan"] = False


def set_fan(on: bool, source: str = "app"):
    """Điều khiển quạt thủ công (từ app) + tạm khoá auto-logic."""
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


def set_led(on: bool, source: str = "app"):
    global _led_manual_until
    if on:
        led_on()
    else:
        led_off()
    _led_manual_until = time.time() + MANUAL_OVERRIDE_SEC
    print(f"[LIVING ROOM] Đèn điều khiển thủ công bởi {source}: {'ON' if on else 'OFF'}")


# ==========================================
# SERVO SG90 - KHOÁ CỬA
# ==========================================
def limit_servo_angle(angle):
    return max(0, min(180, angle))


def _angle_to_duty(angle: float) -> float:
    angle = limit_servo_angle(angle)
    return SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)


def _set_servo_angle(angle: float):
    if _servo_pwm is None:
        return
    with _servo_lock:
        _servo_pwm.ChangeDutyCycle(_angle_to_duty(angle))
        time.sleep(SERVO_MOVE_SEC)
        _servo_pwm.ChangeDutyCycle(0)   # nhả xung để servo không giật/nóng


def lock_door():
    global _unlock_generation
    _unlock_generation += 1            # huỷ mọi luồng auto-relock đang chờ
    _set_servo_angle(SERVO_LOCK_ANGLE)
    state["door_locked"] = True
    print("[DOOR] Đã khoá cửa.")
    _emit_door_status()


def unlock_door(source: str = "app", auto_relock: bool = True):
    """Mở khoá cửa.
    auto_relock=True : tự khoá lại sau AUTO_LOCK_AFTER_SEC giây (app/face).
    auto_relock=False: giữ mở (thoát hiểm khi cháy/gas).
    """
    global _last_authorized_unlock_ts, _unlock_generation
    _unlock_generation += 1
    my_generation = _unlock_generation

    _set_servo_angle(SERVO_UNLOCK_ANGLE)
    state["door_locked"] = False
    _last_authorized_unlock_ts = time.time()
    print(f"[DOOR] Mở khoá bởi: {source} (auto_relock={auto_relock})")
    _emit_door_status()

    if not auto_relock:
        return

    def _auto_relock():
        time.sleep(AUTO_LOCK_AFTER_SEC)

        # Cửa vẫn đang mở thì chờ đóng lại rồi mới khoá (tránh chốt kẹt vào khung)
        waited = 0.0
        while state["door_open"] and waited < DOOR_OPEN_WAIT_MAX:
            time.sleep(0.5)
            waited += 0.5

        # Chỉ khoá nếu không có lệnh mới hơn và không đang báo cháy/gas
        if (
            my_generation == _unlock_generation
            and not state["door_locked"]
            and not state["fire_gas_alarm"]
        ):
            lock_door()

    threading.Thread(target=_auto_relock, daemon=True).start()


# ==========================================
# CẢM BIẾN TỪ CỬA (REED)
# ==========================================
def _apply_door_state(is_open: bool):
    if state["door_open"] == is_open:
        return

    state["door_open"] = is_open
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] [DOOR] {'Cửa đang mở.' if is_open else 'Cửa đã đóng.'}")
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

    _btn_override["led"] = False     # báo cháy/gas: bỏ ghi đè của nút
    _btn_override["fan"] = False

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
    # [SỬA] Luôn tắt còi. Trước đây chỉ tắt khi cửa đóng, nhưng lúc cháy cửa thoát hiểm
    # được mở nên còi sẽ kêu mãi sau khi báo động đã kết thúc.
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    _emit_event("fire_gas", {
        "type": "fire_gas",
        "level": "CLEAR",
        "ts": datetime.now().isoformat(),
    })


# ==========================================
# NÚT NHẤN: nhấn ngắn = bật/tắt (manual), giữ 2 s = về tự động
# Đèn trên CẢ 2 nút sáng khi đèn chiếu sáng TẮT
# ==========================================
class _Button:
    def __init__(self, pin):
        self.pin = pin
        self.stable = False          # True = đang nhấn
        self.last_reading = False
        self.last_change = 0.0
        self.press_start = 0.0
        self.long_fired = False

    def read(self, now):
        """Trả về 'short', 'long' hoặc None."""
        reading = GPIO.input(self.pin) == GPIO.LOW
        if reading != self.last_reading:
            self.last_reading = reading
            self.last_change = now
        if now - self.last_change > BTN_DEBOUNCE_SEC and reading != self.stable:
            self.stable = reading
            if self.stable:
                self.press_start = now
                self.long_fired = False
            elif not self.long_fired:
                return "short"
        if self.stable and not self.long_fired and now - self.press_start >= BTN_LONG_PRESS_SEC:
            self.long_fired = True
            return "long"
        return None


def _update_button_lamps():
    lamp = GPIO.LOW if state["led"] else GPIO.HIGH
    GPIO.output(BTN_LED_SIG1, lamp)
    GPIO.output(BTN_FAN_SIG1, lamp)


def _blink_button_lamps():
    for _ in range(3):
        GPIO.output(BTN_LED_SIG1, GPIO.HIGH)
        GPIO.output(BTN_FAN_SIG1, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(BTN_LED_SIG1, GPIO.LOW)
        GPIO.output(BTN_FAN_SIG1, GPIO.LOW)
        time.sleep(0.1)


def _button_loop():
    global _fan_manual_until, _led_manual_until, _last_btn_ts
    buttons = {"led": _Button(BTN_LED_SIG2), "fan": _Button(BTN_FAN_SIG2)}
    on_fn = {"led": led_on, "fan": fan_on}
    off_fn = {"led": led_off, "fan": fan_off}

    while not _stop_flag:
        now = time.time()

        for dev, btn in buttons.items():
            ev = btn.read(now)
            if ev is None:
                continue

            # An toàn: đang báo cháy/gas thì bỏ qua nút
            if state["fire_gas_alarm"]:
                print(f"[BTN] Đang báo cháy/gas, bỏ qua nút {dev}.")
                continue

            if ev == "short":
                new_on = not state[dev]
                (on_fn if new_on else off_fn)[dev]()
                _btn_override[dev] = True
                _last_btn_ts = now
                print(f"[BTN] {dev} -> {'ON' if new_on else 'OFF'} (MANUAL)")
                _emit_event("device_state", {"device": dev, "on": new_on})  # đồng bộ app
            else:  # long
                _btn_override[dev] = False
                if dev == "led":
                    _led_manual_until = 0.0     # xoá luôn khoá thủ công từ app
                else:
                    _fan_manual_until = 0.0
                print(f"[BTN] {dev} -> AUTO")
                _blink_button_lamps()

        # Tự thả ghi đè khi không còn chuyển động đủ lâu
        if (BTN_OVERRIDE_RELEASE_SEC > 0 and any(_btn_override.values())
                and now - max(state["last_motion_ts"], _last_btn_ts) >= BTN_OVERRIDE_RELEASE_SEC):
            print("[BTN] Không có người lâu: nút trả về AUTO")
            _btn_override["led"] = _btn_override["fan"] = False

        _update_button_lamps()
        time.sleep(0.02)


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
            lux = (data[0] << 8 | data[1]) / 1.2
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
        t, h = _dht.temperature, _dht.humidity
        if t is None or h is None:
            return None, None
        return round(t, 1), round(h, 1)
    except RuntimeError:
        return None, None


# ==========================================
# VÒNG LẶP CHÍNH CỦA PHÒNG KHÁCH
# ==========================================
def run():
    """Gọi hàm này trong 1 thread riêng từ gateway.py."""
    setup_gpio()
    _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)
    lock_door()

    global _btn_thread
    _btn_thread = threading.Thread(target=_button_loop, daemon=True)
    _btn_thread.start()

    print("[LIVING ROOM] Sẵn sàng.")
    last_sensor_read = 0.0

    while not _stop_flag:
        now = time.time()
        _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)   # poll dự phòng

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

            # Quạt: theo hiện diện, giữ 60 s sau lần PIR cuối (nhường nút/app và giữ bật khi báo động)
            if (time.time() >= _fan_manual_until and not _btn_override["fan"]
                    and not state["fire_gas_alarm"]):
                if occupied and not state["fan"]:
                    fan_on()
                elif not occupied and state["fan"]:
                    fan_off()

            # Đèn: tối + có người (nhường nút/app và giữ bật khi báo động)
            if (state["lux"] is not None and time.time() >= _led_manual_until
                    and not _btn_override["led"] and not state["fire_gas_alarm"]):
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
                  f"Đèn: {'ON' if state['led'] else 'OFF'}  "
                  f"Cửa: {'Mở' if state['door_open'] else 'Đóng'}/"
                  f"{'Khoá' if state['door_locked'] else 'Chưa khoá'}")
            _emit_event("sensor_reading", {
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
    if _btn_thread is not None:
        _btn_thread.join(timeout=1)
    try:
        fan_off()
        led_off()
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        GPIO.output(BTN_LED_SIG1, GPIO.LOW)
        GPIO.output(BTN_FAN_SIG1, GPIO.LOW)
        _servo_pwm.stop()
        _fan_pwm.stop()
        GPIO.cleanup()
    except Exception:
        pass


# ==========================================
# TEST SERVO ĐỘC LẬP:  python3 living_room.py
# ==========================================
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    _servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
    _servo_pwm.start(0)
    try:
        while True:
            for a in (SERVO_LOCK_ANGLE, SERVO_UNLOCK_ANGLE):
                print(f"Quay tới {a}°")
                _set_servo_angle(a)
                time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    finally:
        _servo_pwm.stop()
        GPIO.cleanup()