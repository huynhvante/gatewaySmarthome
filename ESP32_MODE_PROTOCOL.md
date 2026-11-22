# Chế độ AUTO/MANUAL riêng từng đèn/quạt – giao thức

## App ↔ Gateway (MQTT) – đã chạy cho cả 3 phòng

| Topic | Payload | Ghi chú |
|---|---|---|
| `smarthome/<room>/led_mode/set` | `AUTO` \| `MANUAL` | app gửi |
| `smarthome/<room>/fan_mode/set` | `AUTO` \| `MANUAL` | app gửi |
| `smarthome/<room>/led_mode/state` | `AUTO` \| `MANUAL` | gateway gửi, retained |
| `smarthome/<room>/fan_mode/state` | `AUTO` \| `MANUAL` | gateway gửi, retained |
| `smarthome/<room>/fan_level/set` | `0` `1` `2` `3` | 0 = tắt; quạt chuyển MANUAL (cả 3 phòng) |
| `smarthome/<room>/fan_level/state` | `0` `1` `2` `3` | retained |

`<room>` = `living_room`, `bedroom`, `kitchen`.

Quy tắc chung:
- `MANUAL`: giữ nguyên trạng thái hiện tại, chỉ không cho tự động đổi nữa.
- `AUTO`: bỏ ghi đè của **riêng thiết bị đó**, đánh giá lại ngay theo cảm biến.
- Bật/tắt tay (app hoặc nút vật lý) → **thiết bị đó** chuyển `MANUAL`.
- Đang báo gas/cháy: gateway từ chối lệnh đèn/quạt/mode.

Mức quạt phòng khách (Pi): `FAN_LEVEL_DUTY = (0, 50, 65, 80)` %. AUTO và báo cháy chạy mức 3 (80%).
Mức quạt phòng ngủ/bếp (ESP32): `constexpr uint8_t FAN_PWM[] = {0, 100, 150, 200};` (PWM 8 bit).

## Gateway ↔ ESP32 (bridge serial) – firmware phòng ngủ/bếp CẦN BỔ SUNG

### 1. Lệnh mới gateway gửi xuống (dạng `<node>:<CMD>\n`)
```
bedroom:LED_AUTO    bedroom:LED_MANUAL    bedroom:FAN_AUTO    bedroom:FAN_MANUAL
kitchen:LED_AUTO    kitchen:LED_MANUAL    kitchen:FAN_AUTO    kitchen:FAN_MANUAL
```
Mức quạt (khi app chọn mức): gateway gửi **2 lệnh liên tiếp**
```
bedroom:FAN_ON       (mức 1..3)   hoặc   bedroom:FAN_OFF   (mức 0)
bedroom:FAN_LEVEL=2  (0..3)
```
- Firmware cũ chưa hiểu `FAN_LEVEL` vẫn bật/tắt được nhờ `FAN_ON`/`FAN_OFF`.
- Firmware mới: `FAN_LEVEL=n` → `fanLevel = n`, `ledcWrite(FAN_PWM[n])`, `fanManual = true`.
  `FAN_ON` khi chưa có mức → dùng mức gần nhất (mặc định 3). AUTO bật quạt → mức 3.

Firmware cần tách cờ `manualMode` chung thành 2 cờ `ledManual`, `fanManual`:
- `LED_ON` / `LED_OFF` → `ledManual = true` (quạt không đổi)
- `FAN_ON` / `FAN_OFF` → `fanManual = true` (đèn không đổi)
- `LED_MANUAL` → `ledManual = true`, giữ nguyên đèn
- `LED_AUTO` → `ledManual = false`, đánh giá auto ngay
- tương tự cho `FAN_*`
- Lệnh `AUTO` cũ (cả hai về auto) vẫn giữ để tương thích.

### 2. Field mới trong JSON telemetry gửi lên
```json
{"node":"bedroom", ..., "led":1, "fan":1, "fan_level":2, "led_mode":"MANUAL", "fan_mode":"AUTO"}
```
- Có các field `fan_level`, `led_mode`, `fan_mode`: gateway báo đúng trạng thái thật lên app.
- Chưa có (firmware cũ): gateway báo lên app theo **lệnh cuối cùng** đã gửi.
