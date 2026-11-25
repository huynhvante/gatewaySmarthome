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
PIR_PIN      = 27 # GPIO27 (chân vật lý 13)
REED_PIN     = 17 # GPIO17 (chân vật lý 11)

SERVO_PIN    = 18      # SG90 - dây tín hiệu vào GPIO18 (chân vật lý 12)
# Phòng khách KHÔNG có còi: báo động gas/cháy kêu bằng còi ở bếp và phòng ngủ (node ESP32).

FAN_INA_PIN  = 23   # GPIO23 (chân vật lý 16)
FAN_INB_PIN  = 19   # GPIO19 (chân vật lý 35) - PWM PHẦN CỨNG PWM0_CHAN3 của chip RP1 (trước đây GPIO24 chân 18)
# INA phải luôn LOW: module driver có điện trở kéo lên, INA bỏ trống = quạt tự quay.
# /boot/firmware/config.txt có "gpio=23=op,dl" để kéo LOW ngay từ lúc Pi khởi động.
FAN_PWM_FREQ = 2000
FAN_SPEED    = 80
# Mức quạt từ app (smarthome/living_room/fan_level/set): 0 = tắt, 1..3 = duty PWM (%).
# Mức 3 = 80% đúng bằng tốc độ cũ; chế độ AUTO luôn chạy mức FAN_AUTO_LEVEL.
FAN_LEVEL_DUTY = (0, 50, 65, 80)
FAN_AUTO_LEVEL = 3
# Nút quạt xanh (giống bedroom): nhấn 1/2/3 lần liên tiếp = mức 1/2/3; nhả tay quá FAN_TAP_GAP_SEC thì chốt.
# Đang chạy tay mà nhấn 1 lần = tắt. Nhấn > 3 lần = bỏ qua. Giữ 2 s = về AUTO.
FAN_TAP_GAP_SEC = 0.5

LED_PIN      = 22   # GPIO22 (chân vật lý 15)
DHT_PIN      = board.D4           # truyền thẳng vào adafruit_dht.DHT11 chân GPIO4 (chân vật lý 7)
I2C_BUS      = 1 # chân vật lý 3/5 (SDA/SCL) 
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
# ---- Đèn: trễ 50/100 lux (giống bedroom) ----
LUX_ON_THRESHOLD  = 50.0     # có người + < 50 lux -> bật đèn
LUX_OFF_THRESHOLD = 100.0    # >= 100 lux trong BRIGHT_SAMPLES_TO_TURN_OFF mẫu liên tiếp -> tắt đèn
BRIGHT_SAMPLES_TO_TURN_OFF = 2
LUX_THRESHOLD     = LUX_ON_THRESHOLD   # giữ tên cũ cho code khác (nếu có) đang dùng
LUX_SAMPLE_SEC    = 0.3      # đọc BH1750 mỗi 300 ms
SENSOR_INTERVAL_SEC = 2.0    # đọc DHT + gửi telemetry định kỳ mỗi 2 s

# ---- Quạt: chỉ bật khi có người VÀ nóng (trễ 28/26 °C, giống bedroom) ----
FAN_TEMP_ON_C  = 28.0        # >= nhiệt độ này thì cho phép bật quạt
FAN_TEMP_OFF_C = 26.0        # <= nhiệt độ này thì tắt quạt

# ---- PIR: lọc + xác nhận có người bằng 2 xung (giống bedroom) ----
PIR_WARMUP_SEC         = 30.0   # bỏ qua PIR 30 s đầu (module đang ổn định)
PIR_STARTUP_LOW_SEC    = 3.0    # sau warmup, PIR phải LOW liên tục 3 s mới READY
PIR_SAMPLE_SEC         = 0.02   # lấy mẫu GPIO mỗi 20 ms
PIR_DEBUG_RAW          = False  # True = in mọi thay đổi RAW của chân PIR
ENTRY_MIN_INTERVAL_SEC = 2.0    # cạnh lên thứ 2 phải cách cạnh đầu >= 2 s
ENTRY_WINDOW_SEC       = 5.0    # ... và <= 5 s, nếu không thì về EMPTY
PRESENCE_HOLD_SEC      = 5.0    # đã OCCUPIED: PIR LOW liên tục 5 s -> EMPTY
MOTION_HOLD_SEC        = PRESENCE_HOLD_SEC   # giữ tên cũ

AUTO_LOCK_AFTER_SEC = 3      # mở khoá 3 giây rồi tự khoá lại (theo đề cương)
DOOR_OPEN_WAIT_MAX  = 5     # nếu cửa vẫn đang mở thì chờ tối đa 5 s rồi mới khoá

# ==========================================
# NÚT NHẤN TÍCH HỢP LED
# SIG2: ngõ ra trạng thái nút (mặc định HIGH, nhấn = LOW)
# SIG1: ngõ vào điều khiển LED nút (mặc định LOW, HIGH = sáng)
# ==========================================
BTN_LED_SIG2 = 5      # nút vàng (đèn) chân vật lý 29
BTN_LED_SIG1 = 6      # nút vàng (đèn) chân vật lý 31
BTN_FAN_SIG2 = 13     # nút xanh (quạt) chân vật lý 33
BTN_FAN_SIG1 = 26     # nút xanh (quạt) chân vật lý 37
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
    "fan_level": 0,          # 0..3
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


class HardwarePWM:
    CHANNEL = 3                      # GPIO19 = PWM0 kênh 3

    def __init__(self, freq_hz: int):
        self.dir = self._find_channel_dir()
        self.period_ns = int(1_000_000_000 / freq_hz)
        self._write("duty_cycle", 0)              # duty phải <= period trước khi đổi period
        self._write("period", self.period_ns)
        self._write("duty_cycle", 0)
        self._write("enable", 1)

    @classmethod
    def _find_channel_dir(cls) -> str:
        base = "/sys/class/pwm"
        chips = sorted(os.listdir(base)) if os.path.isdir(base) else []
        # PWM0 của RP1 (GPIO12/13/18/19) có địa chỉ ...98000; PWM1 là quạt tản nhiệt của Pi 5 -> KHÔNG đụng vào
        for c in chips:
            if "98000" in os.path.realpath(os.path.join(base, c)):
                d = os.path.join(base, c, f"pwm{cls.CHANNEL}")
                if os.path.isdir(d):
                    return d
        raise RuntimeError("chưa có kênh PWM0/pwm3 (chạy fan-pwm-setup.sh hoặc kiểm tra gateway.service)")

    def _write(self, name: str, value: int):
        with open(os.path.join(self.dir, name), "w") as f:
            f.write(str(int(value)))

    def start(self, duty: float):
        self.ChangeDutyCycle(duty)

    def ChangeDutyCycle(self, duty: float):
        duty = max(0.0, min(100.0, float(duty)))
        self._write("duty_cycle", self.period_ns * duty / 100.0)

    def stop(self):
        self._write("duty_cycle", 0)              # giữ kênh bật, chân ở mức thấp
_stop_flag = False
_last_lux_warn_ts = 0.0
_event_callback = None

# Quạt ramp: tránh sụt áp khi bật quạt từ 0 -> 100% duty cycle
_fan_ramp_stop = threading.Event()
_fan_ramp_thread = None
FAN_RAMP_TIME = 1.5   
FAN_RAMP_STEPS = 20    

# Manual từ app: RIÊNG cho từng thiết bị, giữ đến khi app gửi AUTO cho thiết bị đó
# (smarthome/living_room/<led|fan>_mode/set). Bật/tắt tay từ app cũng chuyển thiết bị đó sang manual.
_app_manual = {"led": False, "fan": False}
_last_fan_level = FAN_AUTO_LEVEL   # mức dùng khi bật quạt bằng công tắc ON/OFF hoặc nút vật lý
# Yêu cầu vòng chính đánh giá auto ngay (giống lastSensorMs = millis() - SENSOR_INTERVAL_MS bên C)
_force_eval = False

# Nút nhấn quyết định thiết bị (ưu tiên hơn tự động)
_btn_override = {"led": False, "fan": False}
_pre_alarm_snapshot = None
_last_alarm_key = None
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

    GPIO.setup(PIR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)   # giống INPUT_PULLDOWN bên ESP
    GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # đóng cửa = mạch kín = LOW
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(FAN_INA_PIN, GPIO.OUT, initial=GPIO.LOW)
    # INB: ưu tiên PWM phần cứng (không tốn CPU). Không được GPIO.setup chân này trước,
    # vì như vậy sẽ chuyển GPIO19 khỏi chức năng PWM.
    try:
        _fan_pwm = HardwarePWM(FAN_PWM_FREQ)
        print(f"[LIVING ROOM] Quạt dùng PWM phần cứng ({_fan_pwm.dir}, {FAN_PWM_FREQ} Hz).")
    except Exception as e:
        print(f"[WARN] Không dùng được PWM phần cứng cho quạt ({e}). "
              "Tạm dùng PWM phần mềm (tốn CPU).")
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

    _dht = adafruit_dht.DHT11(DHT_PIN, use_pulseio=False)

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

    # PIR không dùng ngắt nữa: _pir_loop() lấy mẫu mỗi 20 ms và chạy bộ lọc giống bedroom.
    GPIO.add_event_detect(REED_PIN, GPIO.BOTH, callback=_on_door_state_change, bouncetime=200)


# ==========================================
# QUẠT
# ==========================================

# điều khiển mở quạt từ từ để tránh sụt áp (INA=LOW, INB=PWM)
_fan_duty = 0.0   # duty PWM hiện tại của quạt (%)


def _fan_ramp(start, target):
    """Tăng/giảm duty từ start tới target trong FAN_RAMP_TIME (tránh sụt áp khi khởi động quạt)."""
    global _fan_duty
    step_delay = FAN_RAMP_TIME / FAN_RAMP_STEPS
    for i in range(1, FAN_RAMP_STEPS + 1):
        if _fan_ramp_stop.is_set():
            return                      # bị ngắt (vd fan_off được gọi giữa chừng)
        _fan_duty = start + (target - start) * i / FAN_RAMP_STEPS
        _fan_pwm.ChangeDutyCycle(_fan_duty)
        time.sleep(step_delay)


_fan_lock = threading.RLock() # tránh nhiều luồng cùng bật/tắt quạt một lúc

def fan_set_level(level: int):
    """Đặt mức quạt 0..3 (0 = tắt). Đổi mức khi đang chạy thì chuyển mượt từ duty hiện tại."""
    global _fan_ramp_thread, _fan_duty
    level = max(0, min(3, int(level)))
    with _fan_lock:
        if level == 0:
            fan_off()
            return
        if state["fan"] and state["fan_level"] == level:
            return                      # đang chạy đúng mức rồi, không ramp lại
        _fan_ramp_stop.set()            # dừng ramp cũ (nếu có)
        if _fan_ramp_thread is not None:
            _fan_ramp_thread.join()
        GPIO.output(FAN_INA_PIN, GPIO.LOW)
        _fan_ramp_stop.clear()
        start = _fan_duty if state["fan"] else 0.0
        _fan_ramp_thread = threading.Thread(target=_fan_ramp, args=(start, FAN_LEVEL_DUTY[level]),
                                            daemon=True)
        _fan_ramp_thread.start()
        state["fan"] = True
        state["fan_level"] = level


def fan_on(level: int = FAN_AUTO_LEVEL):
    fan_set_level(level)


def fan_off():
    global _fan_duty
    with _fan_lock:
        _fan_ramp_stop.set()            # dừng ramp đang chạy (nếu có)
        if _fan_ramp_thread is not None:
            _fan_ramp_thread.join()
        GPIO.output(FAN_INA_PIN, GPIO.LOW)
        _fan_pwm.ChangeDutyCycle(0)
        _fan_duty = 0.0
        state["fan"] = False
        state["fan_level"] = 0


def set_fan(on: bool, source: str = "app"):
    """Bật/tắt quạt thủ công từ app (công tắc ON/OFF): quạt chuyển sang MANUAL.
    Bật lại thì dùng mức gần nhất người dùng đã chọn."""
    fan_set_level(_last_fan_level if on else 0)
    _app_manual["fan"] = True
    print(f"[LIVING ROOM] Quạt điều khiển thủ công bởi {source}: {'ON' if on else 'OFF'} (MANUAL)")


def set_fan_level_manual(level: int, source: str = "app"):
    """Chọn mức quạt 0..3 từ app (fan_level/set): quạt chuyển sang MANUAL."""
    global _last_fan_level
    level = max(0, min(3, int(level)))
    if level > 0:
        _last_fan_level = level
    fan_set_level(level)
    _app_manual["fan"] = True
    print(f"[LIVING ROOM] Mức quạt {level} ({FAN_LEVEL_DUTY[level]}%) bởi {source} (MANUAL)")


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
    """Bật/tắt đèn thủ công từ app: đèn chuyển sang MANUAL (quạt không bị ảnh hưởng)."""
    if on:
        led_on()
    else:
        led_off()
    _app_manual["led"] = True
    print(f"[LIVING ROOM] Đèn điều khiển thủ công bởi {source}: {'ON' if on else 'OFF'} (MANUAL)")


def get_mode(device: str) -> str:
    """'MANUAL' nếu thiết bị đang do người dùng quyết định (app hoặc nút vật lý), ngược lại 'AUTO'."""
    return "MANUAL" if (_app_manual[device] or _btn_override[device]) else "AUTO"


def set_mode(device: str, mode: str, source: str = "app"):
    """Đổi chế độ 1 thiết bị (led/fan) từ app.
    MANUAL: giữ nguyên trạng thái hiện tại, chỉ không cho tự động đổi nữa.
    AUTO  : bỏ mọi ghi đè của thiết bị đó, vòng điều khiển đánh giá lại ngay theo cảm biến."""
    global _force_eval
    if device not in ("led", "fan") or mode not in ("AUTO", "MANUAL"):
        raise ValueError(f"set_mode: {device}={mode}")
    if mode == "MANUAL":
        _app_manual[device] = True
    else:
        _app_manual[device] = False
        _btn_override[device] = False
        _force_eval = True
    print(f"[LIVING ROOM] {device} -> {mode} bởi {source}")


def set_auto(source: str = "app"):
    """Tương đương lệnh AUTO / ALARM_OFF bên C: đưa CẢ đèn và quạt về AUTO, tắt đèn/quạt.
    Đang báo cháy/gas thì không tắt thiết bị (giữ điểm an toàn riêng của Python)."""
    global _force_eval
    _app_manual["led"] = _app_manual["fan"] = False
    _btn_override["led"] = False
    _btn_override["fan"] = False
    _force_eval = True
    if not state["fire_gas_alarm"]:
        fan_off()
        led_off()
    print(f"[LIVING ROOM] Về chế độ AUTO bởi {source}")


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
# Tầng 1: lọc GPIO (WARMUP -> WAIT_LOW -> READY), giống updatePirFilter() bên bedroom.
class _PirFilter:
    WARMUP, WAIT_LOW, READY = "WARMUP", "WAIT_LOW", "READY"

    def __init__(self, now):
        self.phase = self.WARMUP
        self.boot = now
        self.raw = -1                 # chưa lấy mẫu
        self.filtered = False
        self.stable_since = now
        self.last_sample = now

    def update(self, now, raw):
        gap = now - self.last_sample
        self.last_sample = now
        changed = raw != self.raw
        self.raw = raw
        if changed and PIR_DEBUG_RAW:
            print(f"[PIR] RAW={raw}")

        # Thread bị trễ (GIL, đọc DHT...) thì xác nhận lại từ mẫu mới.
        if changed or gap > 3 * PIR_SAMPLE_SEC:
            self.stable_since = now

        if self.phase == self.WARMUP:
            if now - self.boot < PIR_WARMUP_SEC:
                return
            self.phase = self.WAIT_LOW
            self.stable_since = now
            print(f"[PIR] WAIT_LOW ({PIR_STARTUP_LOW_SEC:.0f} s)")

        if self.phase == self.WAIT_LOW:
            if raw == GPIO.LOW and now - self.stable_since >= PIR_STARTUP_LOW_SEC:
                self.phase = self.READY
                print("[PIR] READY")
            return

        high = raw == GPIO.HIGH
        if high != self.filtered:
            self.filtered = high
            print(f"[PIR] FLT={1 if high else 0}")


# Tầng 2: xác nhận có người, giống updateRoomPresence() bên bedroom.
#   EMPTY --cạnh lên--> WAIT_SECOND --cạnh lên sau 2-5 s--> OCCUPIED
#   WAIT_SECOND --quá 5 s--> EMPTY ;  OCCUPIED --FLT=0 liên tục 5 s--> EMPTY
class _RoomPresence:
    EMPTY, WAIT_SECOND, OCCUPIED = "EMPTY", "WAIT_SECOND", "OCCUPIED"

    def __init__(self):
        self.status = self.EMPTY
        self.first_pulse = 0.0
        self.prev_motion = False
        self.last_edge = 0.0

    def update(self, motion, now):
        since_edge = now - self.last_edge

        # Xét hết hạn trước khi xử lý cạnh mới.
        if self.status == self.WAIT_SECOND and now - self.first_pulse > ENTRY_WINDOW_SEC:
            self.status = self.EMPTY
            print("[ROOM] EMPTY (entry window expired)")
        elif (self.status == self.OCCUPIED and not self.prev_motion
              and since_edge >= PRESENCE_HOLD_SEC):
            self.status = self.EMPTY
            print(f"[ROOM] EMPTY (FLT=0 for {since_edge:.1f} s)")

        if motion != self.prev_motion:
            self.prev_motion = motion
            self.last_edge = now
            state["last_motion_ts"] = time.time()   # dùng cho bộ đếm thả ghi đè nút
            if motion and self.status == self.EMPTY:
                self.first_pulse = now
                self.status = self.WAIT_SECOND
                print("[ROOM] WAIT_SECOND")
            elif (motion and self.status == self.WAIT_SECOND
                  and now - self.first_pulse >= ENTRY_MIN_INTERVAL_SEC):
                self.status = self.OCCUPIED
                print("[ROOM] OCCUPIED")
            elif motion and self.status == self.OCCUPIED:
                print(f"[ROOM] LOW_GAP={since_edge:.1f} s")

        return self.status == self.OCCUPIED


_pir = None                          # _PirFilter (để in log)
_presence = False                    # kết quả tầng 2, vòng chính đọc
_presence_event = threading.Event()  # báo vòng chính khi _presence đổi -> phản ứng ngay
_pir_thread = None


def _pir_loop():
    global _pir, _presence
    now = time.monotonic()
    _pir = _PirFilter(now)
    room = _RoomPresence()
    print(f"[PIR] WARMUP {PIR_WARMUP_SEC:.0f} s")
    while not _stop_flag:
        now = time.monotonic()
        _pir.update(now, GPIO.input(PIR_PIN))
        occupied = room.update(_pir.filtered, now)
        if occupied != _presence:
            _presence = occupied
            _presence_event.set()
        time.sleep(PIR_SAMPLE_SEC)


def _is_occupied() -> bool:
    """Giữ tên cũ: trả về trạng thái có người đã xác nhận."""
    return _presence


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
    global _pre_alarm_snapshot, _last_alarm_key 
    if not state["fire_gas_alarm"]:
        _pre_alarm_snapshot = {
            "app_manual": dict(_app_manual),
            "btn_override": dict(_btn_override),
            "fan": state["fan"],
            "fan_level": state["fan_level"],
            "led": state["led"],
        }
    
    state["fire_gas_alarm"] = True
    state["fire_gas_level"] = level

    _btn_override["led"] = False     # báo cháy/gas: bỏ ghi đè của nút
    _btn_override["fan"] = False

    # Báo cháy/gas: phần cứng phòng khách chỉ MỘT mức (mọi level như nhau):
    # quạt mức cao nhất để thông gió + bật đèn. Không còi, không tự mở cửa.
    fan_on(3)
    led_on()

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
    global _pre_alarm_snapshot, _btn_override, _force_eval, _last_alarm_key, _last_btn_ts
    snap = _pre_alarm_snapshot
    _pre_alarm_snapshot = None
    _last_alarm_key = None
    
    state["fire_gas_alarm"] = False
    state["fire_gas_level"] = "CLEAR"
    # Giống ALARM_OFF bên C: khôi phục đèn/quạt như trước khi báo động
    
    if snap is None:
        set_auto("alarm_clear")
    else:
        _app_manual.update(snap["app_manual"])
        _btn_override["fan"] = snap["btn_override"]["fan"]
        _btn_override["led"] = snap["btn_override"]["led"]

        # Thiết bị đang MANUAL: trả về đúng trạng thái (và mức quạt) trước khi báo cháy
        if get_mode("fan") == "MANUAL":
            fan_set_level(snap["fan_level"] if snap["fan"] else 0)
        if get_mode("led") == "MANUAL":
            (led_on if snap["led"] else led_off)()
                    
        if any(_btn_override.values()):
            _last_btn_ts = time.time()  # reset thời gian ghi đè nút để tránh tự thả quá sớm
        _force_eval = True  # đánh giá auto ngay
        print("[LIVING ROOM] Hết báo cháy/gas: khôi phục trạng thái trước đó.")
        
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

# Hàm nhấp nháy đèn nút khi chuyển sang chế độ AUTO mỗi nút điều khiển đèn
def _blink_button_lamps_led():
    for _ in range(3):
        GPIO.output(BTN_LED_SIG1, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(BTN_LED_SIG1, GPIO.LOW)
        time.sleep(0.1)
    _update_button_lamps()
    
# Hàm nhấp nháy đèn nút khi chuyển sang chế độ AUTO mỗi nút điều khiển quạt
def _blink_button_lamps_fan():
    for _ in range(3):
        GPIO.output(BTN_FAN_SIG1, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(BTN_FAN_SIG1, GPIO.LOW)
        time.sleep(0.1)
    _update_button_lamps()

def _button_loop():
    global _last_btn_ts, _force_eval, _last_fan_level
    buttons = {"led": _Button(BTN_LED_SIG2), "fan": _Button(BTN_FAN_SIG2)}
    fan_taps = 0              # số lần nhấn ngắn nút xanh trong chuỗi hiện tại
    last_fan_release = 0.0

    while not _stop_flag:
        now = time.time()

        for dev, btn in buttons.items():
            ev = btn.read(now)
            if ev is None:
                continue

            # An toàn: đang báo cháy/gas thì bỏ qua nút
            if state["fire_gas_alarm"]:
                if dev == "fan":
                    fan_taps = 0
                print(f"[BTN] Đang báo cháy/gas, bỏ qua nút {dev}.")
                continue

            if ev == "short" and dev == "led":
                new_on = not state["led"]
                led_on() if new_on else led_off()
                _btn_override["led"] = True
                _last_btn_ts = now
                print(f"[BTN] led -> {'ON' if new_on else 'OFF'} (MANUAL)")
                _emit_event("device_state", {"device": "led", "on": new_on})  # đồng bộ app
            elif ev == "short":
                # Nút xanh: chỉ đếm, chốt mức sau khi nhả tay FAN_TAP_GAP_SEC (giống bedroom)
                fan_taps = min(fan_taps + 1, 5)
                last_fan_release = now
            else:  # long
                _btn_override[dev] = False
                _app_manual[dev] = False    # giữ nút 2 s = thiết bị NÀY về AUTO (kể cả manual từ app)
                _force_eval = True          # đánh giá auto ngay
                print(f"[BTN] {dev} -> AUTO")
                if dev == "led":
                    _blink_button_lamps_led()
                    _emit_event("device_state", {"device": "led", "on": state["led"]})
                else:
                    fan_taps = 0            # giữ nút huỷ chuỗi nhấn đang đếm
                    _blink_button_lamps_fan()
                    _emit_event("device_state", {"device": "fan", "on": state["fan"],
                                                 "level": state["fan_level"]})
                    _emit_event("fan_button", {"value": 255})   # 255 = giữ nút về AUTO (giống bedroom)

        # Chốt chuỗi nhấn nút xanh: đã nhả hẳn và quá FAN_TAP_GAP_SEC không nhấn thêm
        fan_btn = buttons["fan"]
        if (fan_taps and not fan_btn.stable and not fan_btn.last_reading
                and now - last_fan_release >= FAN_TAP_GAP_SEC):
            taps, fan_taps = fan_taps, 0
            if state["fire_gas_alarm"]:
                pass
            elif taps <= 3:
                fan_manual = _btn_override["fan"] or _app_manual["fan"]
                turn_off = taps == 1 and fan_manual and state["fan_level"] > 0
                level = 0 if turn_off else taps
                if level > 0:
                    _last_fan_level = level     # app bật lại (ON) sẽ dùng mức này
                fan_set_level(level)
                _btn_override["fan"] = True
                _last_btn_ts = now
                print(f"[BTN] fan -> mức {state['fan_level']} ({taps} lần nhấn, MANUAL)")
                _emit_event("device_state", {"device": "fan", "on": state["fan"],
                                             "level": state["fan_level"]})   # đồng bộ app
                _emit_event("fan_button", {"value": taps})                  # 1/2/3 = số lần nhấn
            else:
                print(f"[BTN] fan: {taps} lần nhấn, bỏ qua")

        # Tự thả ghi đè nút khi không còn chuyển động đủ lâu (giữ nguyên manual đặt từ app)
        if (BTN_OVERRIDE_RELEASE_SEC > 0 and any(_btn_override.values()) and not _presence
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
    except Exception as e:          # lỗi khác: không để sập thread
        print(f"[WARN] DHT lỗi: {e}")
        return None, None

# ==========================================
# VÒNG LẶP CHÍNH CỦA PHÒNG KHÁCH
# ==========================================
def run():
    """Gọi hàm này trong 1 thread riêng từ gateway.py."""
    global _btn_thread, _pir_thread, _force_eval

    setup_gpio()
    _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)
    lock_door()

    _btn_thread = threading.Thread(target=_button_loop, daemon=True)
    _btn_thread.start()
    _pir_thread = threading.Thread(target=_pir_loop, daemon=True)
    _pir_thread.start()

    print("[LIVING ROOM] Sẵn sàng.")
    last_sensor_read = float("-inf")
    last_lux_read = float("-inf")
    last_presence = False
    last_lux_ok = False      # lần đọc BH1750 gần nhất có thành công không
    bright_samples = 0       # số mẫu lux >= LUX_OFF_THRESHOLD liên tiếp (tối đa BRIGHT_SAMPLES_TO_TURN_OFF)
    temp_hot = False         # trạng thái nóng có trễ 28/26 °C

    while not _stop_flag:
        # Ngủ tối đa 50 ms, nhưng thức ngay khi tầng PIR báo presence đổi.
        _presence_event.wait(0.05)
        _presence_event.clear()

        now = time.monotonic()
        _apply_door_state(GPIO.input(REED_PIN) == GPIO.HIGH)   # poll dự phòng

        # 1. Hiện diện đã xác nhận (tầng 1 + 2 chạy trong _pir_loop)
        presence = _presence
        presence_changed = presence != last_presence
        last_presence = presence
        state["motion"] = presence

        force = _force_eval
        if force:
            _force_eval = False
        sensor_due = now - last_sensor_read >= SENSOR_INTERVAL_SEC
        lux_due = now - last_lux_read >= LUX_SAMPLE_SEC
        if not (sensor_due or lux_due or presence_changed or force):
            continue

        # 2. DHT mỗi 2 s, BH1750 mỗi 300 ms
        if sensor_due:
            last_sensor_read = now
            temp, humid = _read_dht()
            if temp is not None:
                state["temp"], state["humid"] = temp, humid
                if temp >= FAN_TEMP_ON_C:
                    temp_hot = True
                elif temp <= FAN_TEMP_OFF_C:
                    temp_hot = False

        if lux_due:
            last_lux_read = now
            lux = _read_lux()
            last_lux_ok = lux is not None
            if last_lux_ok:
                state["lux"] = lux
            if last_lux_ok and lux >= LUX_OFF_THRESHOLD:
                bright_samples = min(bright_samples + 1, BRIGHT_SAMPLES_TO_TURN_OFF)
            else:
                bright_samples = 0

        # 3. Điều khiển (nhường manual app, nút nhấn và báo cháy/gas)
        led_before, fan_before = state["led"], state["fan"]
        if not state["fire_gas_alarm"]:
            # Quạt: có người + nóng -> bật; còn lại -> tắt. Xét mỗi 2 s hoặc ngay khi presence đổi.
            if (get_mode("fan") == "AUTO"
                    and (sensor_due or presence_changed or force)):
                if presence and temp_hot:
                    fan_on(FAN_AUTO_LEVEL)
                elif state["fan"]:
                    fan_off()

            # Đèn: bật khi có người + < 50 lux; tắt khi hết người hoặc sáng >= 100 lux 2 mẫu liền.
            if get_mode("led") == "AUTO":
                if state["led"] and (not presence
                                     or bright_samples >= BRIGHT_SAMPLES_TO_TURN_OFF):
                    led_off()
                elif (presence and not state["led"] and last_lux_ok
                      and state["lux"] < LUX_ON_THRESHOLD):
                    led_on()

        # 4. Log + gửi gateway: định kỳ 2 s, hoặc ngay khi có thay đổi
        if not (sensor_due or presence_changed or force
                or led_before != state["led"] or fan_before != state["fan"]):
            continue

        ts = datetime.now().strftime("%H:%M:%S")
        pir_phase = _pir.phase if _pir else "-"
        pir_raw = _pir.raw if _pir else -1
        pir_flt = int(_pir.filtered) if _pir else 0
        print(f"[{ts}] LIVING ROOM | "
              f"Nhiệt độ: {state['temp']}°C  Độ ẩm: {state['humid']}%  "
              f"Ánh sáng: {state['lux']} lux  "
              f"Người: {'Có' if presence else 'Không'}  "
              f"Quạt: {'ON' if state['fan'] else 'OFF'}  "
              f"Đèn: {'ON' if state['led'] else 'OFF'}  "
              f"Cửa: {'Mở' if state['door_open'] else 'Đóng'}/"
              f"{'Khoá' if state['door_locked'] else 'Chưa khoá'}  "
              f"PIR={pir_phase} RAW={pir_raw} FLT={pir_flt} HOT={int(temp_hot)} "
              f"bright={bright_samples}/{BRIGHT_SAMPLES_TO_TURN_OFF} "
              f"mode=led:{get_mode('led')}/fan:{get_mode('fan')} lv={state['fan_level']}"
              )
        _emit_event("sensor_reading", {
            "ts": ts,
            "node": "living_room",
            "temp": state["temp"],
            "humid": state["humid"],
            "lux": state["lux"],
            "motion": presence,
            "fan": state["fan"],
            "fan_level": state["fan_level"],
            "led": state["led"],
            "door_open": state["door_open"],
            "door_locked": state["door_locked"],
        })


def stop():
    global _stop_flag
    _stop_flag = True
    _presence_event.set()   # đánh thức vòng chính để thoát nhanh
    if _btn_thread is not None:
        _btn_thread.join(timeout=1)
    if _pir_thread is not None:
        _pir_thread.join(timeout=1)
    try:
        fan_off()
        led_off()
        GPIO.output(BTN_LED_SIG1, GPIO.LOW)
        GPIO.output(BTN_FAN_SIG1, GPIO.LOW)
        _servo_pwm.stop()
        _fan_pwm.stop()
        GPIO.output(FAN_INA_PIN, GPIO.LOW)
        # KHÔNG trả INA về input: GPIO.cleanup() mặc định đưa mọi chân về input,
        # INA bỏ trống bị điện trở kéo lên của module -> quạt tự quay khi gateway dừng/khởi động lại.
        GPIO.cleanup([p for p in (PIR_PIN, REED_PIN, LED_PIN, SERVO_PIN,
                                  BTN_LED_SIG2, BTN_FAN_SIG2, BTN_LED_SIG1, BTN_FAN_SIG1)])
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
        