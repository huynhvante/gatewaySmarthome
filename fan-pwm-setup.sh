#!/bin/sh
# Chuẩn bị PWM phần cứng cho quạt phòng khách trên Raspberry Pi 5 (chạy bằng root, trước khi gateway khởi động).
# Gọi từ gateway.service:  ExecStartPre=+/usr/local/sbin/fan-pwm-setup.sh hvt
# - GPIO19 (chân 35) -> PWM0_CHAN3 của chip RP1 (INB quạt)
# - GPIO23 (chân 16) -> output LOW (INA quạt; bỏ trống thì module driver kéo lên và quạt tự quay)
# - Tạo kênh pwm3 và cấp quyền ghi cho user chạy gateway
# Luôn thoát với mã 0: nếu lỗi, gateway vẫn chạy và tự dùng PWM phần mềm.
USER_NAME="${1:-hvt}"

pinctrl set 23 op dl || echo "[fan-pwm] Không đặt được GPIO23 LOW"
pinctrl set 19 a3    || echo "[fan-pwm] Không chuyển được GPIO19 sang PWM"

CHIP=""
for c in /sys/class/pwm/pwmchip*; do
  case "$(readlink -f "$c")" in
    *98000*) CHIP="$c" ;;            # PWM0 của RP1. PWM1 (quạt tản nhiệt của Pi 5) không đụng tới.
  esac
done
if [ -z "$CHIP" ]; then
  echo "[fan-pwm] Không tìm thấy PWM0 của RP1 -> gateway sẽ dùng PWM phần mềm"
  exit 0
fi

[ -d "$CHIP/pwm3" ] || echo 3 > "$CHIP/export" 2>/dev/null
i=0
while [ ! -e "$CHIP/pwm3/duty_cycle" ] && [ $i -lt 20 ]; do sleep 0.1; i=$((i + 1)); done
if [ ! -e "$CHIP/pwm3/duty_cycle" ]; then
  echo "[fan-pwm] Không tạo được kênh pwm3 -> gateway sẽ dùng PWM phần mềm"
  exit 0
fi

echo 0 > "$CHIP/pwm3/duty_cycle" 2>/dev/null     # chắc chắn quạt đứng yên khi gateway chưa chạy
chown "$USER_NAME" "$CHIP/pwm3/period" "$CHIP/pwm3/duty_cycle" "$CHIP/pwm3/enable"
echo "[fan-pwm] Sẵn sàng: $CHIP/pwm3 (GPIO19), quyền ghi cho $USER_NAME"
exit 0
