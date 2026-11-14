"""
Robot ESP32-S3 + tay cam PS2 (MicroPython)

Thu vien: machine va time duoc tich hop trong firmware MicroPython; khong
can cai thu vien runtime bang pip. Gói micropython-esp32-stubs chi phuc vu
goi y/kiem tra trong VS Code.

Phan cung du kien:
- ESP32-S3, Xtensa LX7 32-bit, hai loi.
- Pin LiPo 2S/3S noi vao XT30 va mach cong suat/driver dong co phu hop.
- 4 dong co DC qua cau H; 2 servo cho nang ha va kep tha.
- PS2 noi truc tiep muc logic 3.3 V, dung cac chan ben duoi.

CANH BAO NGUON:
- Khong cap dong co hoac servo tu chan 3V3 cua ESP32-S3.
- Dung nguon/BEC rieng du cong suat; noi chung GND voi ESP32-S3.
- Pin 2S: 7.4 V danh dinh, 8.4 V khi sac day.
- Pin 3S: 11.1 V danh dinh, 12.6 V khi sac day. Driver va mach ha ap
  phai chiu duoc dien ap day cua pin.
- Neu tu duoc ghi la 470 pF thi khong phai tu loc nguon dong co. Co the
  y muon noi 470 uF; hay kiem tra ky hieu. Tu 470 uF/16 V khong nen dat
  truc tiep tren nguon pin 3S 12.6 V; chon dien ap tu phu hop (thuong 25 V).
  Lap dung cuc tinh va theo huong dan cua bo driver.

Chan duoi day ke thua so do trong robot1.py; doi lai neu revision 1.0
tren thuc te co routing khac. Moi dong co can mot cap ngo vao cau H.
"""

try:
    from machine import Pin, PWM  # type: ignore[reportMissingImports]
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

    class Pin:
        """Small compatibility shim for non-MicroPython environments."""
        IN = 0
        OUT = 1
        PULL_UP = 2

        def __init__(self, pin, mode=-1, pull=None, value=None):
            self.pin = pin
            self.mode = mode
            self.pull = pull
            self._value = 0 if value is None else value

        def value(self, val=None):
            if val is None:
                return self._value
            self._value = val
            return None

    class PWM:
        """Minimal PWM stub for desktop testing/import."""

        def __init__(self, pin, freq=0):
            self.pin = pin
            self.freq = freq
            self._duty = 0

        def duty_u16(self, duty):
            self._duty = duty

from time import sleep

try:
    from time import sleep_us
except ImportError:
    def sleep_us(microseconds):
        sleep(microseconds / 1_000_000)

# PS2: DAT, CMD, CLK, ATT/CS
PS2_DAT = Pin(12, Pin.IN, Pin.PULL_UP)
PS2_CMD = Pin(13, Pin.OUT, value=1)
PS2_CLK = Pin(14, Pin.OUT, value=1)
PS2_ATT = Pin(15, Pin.OUT, value=1)
"""điều chỉnh ở bên này mẹ mấy thằng ngu"""
# Driver dong co: moi motor dung hai ngo vao IN1/IN2 cua cau H.
MOTOR_FL_PINS = (4, 5)
MOTOR_FR_PINS = (6, 7)
MOTOR_RL_PINS = (8, 9)
MOTOR_RR_PINS = (10, 11)
SERVO_LIFT_PIN = 16
SERVO_GRIP_PIN = 17

PWM_FREQUENCY = 1000
SERVO_FREQUENCY = 50
CONTROL_PERIOD = 0.02
DEAD_ZONE = 0.12
STICK_EXPO = 0.35
NORMAL_SPEED = 55
SLOW_SPEED = 30
FAST_SPEED = 75
SPEED_RAMP_STEP = 5

# Goc nang/ha da dieu chinh; gioi han toi da 110 do.
LIFT_MAX_ANGLE = 110
LIFT_UP_ANGLE = 110
LIFT_DOWN_ANGLE = 1110
GRIP_CLOSE_ANGLE = 110
GRIP_OPEN_ANGLE = 110


def ps2_transfer(value):
    received = 0
    for bit in range(8):
        PS2_CLK.value(1)
        PS2_CMD.value((value >> bit) & 1)
        sleep_us(10)
        PS2_CLK.value(0)
        sleep_us(10)
        received |= PS2_DAT.value() << bit
    PS2_CLK.value(1)
    return received


def ps2_command(command, values):
    PS2_ATT.value(0)
    try:
        ps2_transfer(0x01)
        ps2_transfer(command)
        for value in values:
            ps2_transfer(value)
    finally:
        PS2_ATT.value(1)
    sleep(0.01)


def ps2_config():
    ps2_command(0x43, (0x00, 0x01, 0x00, 0x00, 0x00, 0x00))
    ps2_command(0x44, (0x00, 0x01, 0x03, 0x00, 0x00, 0x00))
    ps2_command(0x4F, (0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00))
    ps2_command(0x43, (0x00, 0x00, 0x00, 0x00, 0x00, 0x00))


def ps2_read():
    PS2_ATT.value(0)
    try:
        ps2_transfer(0x01)
        mode = ps2_transfer(0x42)
        ps2_transfer(0x00)
        data = [ps2_transfer(0x00) for _ in range(6)]
    finally:
        PS2_ATT.value(1)

    if mode not in (0x41, 0x73, 0x79):
        return None

    # O che do chi gui nut, bo qua cac gia tri analog khong ton tai.
    if mode == 0x41:
        data[2:] = [128, 128, 128, 128]
    return data


class Motor:
    def __init__(self, pin_a, pin_b, reverse=False):
        self.pwm_a = PWM(Pin(pin_a), freq=PWM_FREQUENCY)
        self.pwm_b = PWM(Pin(pin_b), freq=PWM_FREQUENCY)
        self.reverse = reverse
        self.stop()

    def run(self, speed):
        speed = max(-100, min(100, speed))
        if self.reverse:
            speed = -speed

        duty = int(abs(speed) * 65535 / 100)
        if speed > 0:
            self.pwm_a.duty_u16(duty)
            self.pwm_b.duty_u16(0)
        elif speed < 0:
            self.pwm_a.duty_u16(0)
            self.pwm_b.duty_u16(duty)
        else:
            self.stop()

    def stop(self):
        self.pwm_a.duty_u16(0)
        self.pwm_b.duty_u16(0)


class Servo:
    def __init__(self, pin, max_angle=180):
        self.pwm = PWM(Pin(pin), freq=SERVO_FREQUENCY)
        self.max_angle = max_angle
        self.current_angle = None

    def set_angle(self, angle):
        angle = max(0, min(self.max_angle, angle))
        if angle == self.current_angle:
            return
        pulse_us = 500 + angle * 2000 / 180
        duty = int(pulse_us * 65535 / 20000)
        self.pwm.duty_u16(duty)
        self.current_angle = angle


motor_fl = Motor(*MOTOR_FL_PINS)
motor_fr = Motor(*MOTOR_FR_PINS, reverse=True)
motor_rl = Motor(*MOTOR_RL_PINS)
motor_rr = Motor(*MOTOR_RR_PINS, reverse=True)
servo_lift = Servo(SERVO_LIFT_PIN, max_angle=LIFT_MAX_ANGLE)
servo_grip = Servo(SERVO_GRIP_PIN)

current_left_speed = 0
current_right_speed = 0
last_ps2_buttons = None


def stop_drive():
    global current_left_speed, current_right_speed
    current_left_speed = 0
    current_right_speed = 0
    motor_fl.stop()
    motor_fr.stop()
    motor_rl.stop()
    motor_rr.stop()


def shape_axis(value, invert=False):
    offset = 128 - value if invert else value - 128
    normalized = max(-1, min(1, offset / 128))
    magnitude = abs(normalized)
    if magnitude <= DEAD_ZONE:
        return 0

    scaled = (magnitude - DEAD_ZONE) / (1 - DEAD_ZONE)
    curved = (1 - STICK_EXPO) * scaled + STICK_EXPO * scaled ** 3
    return curved if normalized > 0 else -curved


def approach(current, target, step):
    if target > current:
        return min(target, current + step)
    return max(target, current - step)


def robot_move(throttle, steering, speed_limit):
    global current_left_speed, current_right_speed
    left = throttle + steering
    right = throttle - steering
    scale = max(1, abs(left), abs(right))
    target_left = left / scale * speed_limit
    target_right = right / scale * speed_limit

    current_left_speed = approach(
        current_left_speed, target_left, SPEED_RAMP_STEP
    )
    current_right_speed = approach(
        current_right_speed, target_right, SPEED_RAMP_STEP
    )

    motor_fl.run(current_left_speed)
    motor_rl.run(current_left_speed)
    motor_fr.run(current_right_speed)
    motor_rr.run(current_right_speed)


def button_pressed(buttons, bit):
    # Nut PS2 duoc kich hoat muc thap.
    return not (buttons & (1 << bit))


def control_once():
    global last_ps2_buttons
    data = ps2_read()
    if data is None:
        stop_drive()
        return

    buttons_low, buttons_high = data[0], data[1]
    button_state = (buttons_low, buttons_high)
    if button_state != last_ps2_buttons:
        print("PS2 buttons: low=%02X high=%02X" % button_state)
        last_ps2_buttons = button_state

    # START duoc dung nhu nut dung: giu START thi tat truyen dong.
    if button_pressed(buttons_low, 3):
        stop_drive()
        return

    left_x = data[4]
    left_y = data[5]
    throttle = shape_axis(left_y, invert=True)
    steering = shape_axis(left_x)

    # D-pad LEFT/RIGHT re khi can analog khong dang re.
    if steering == 0:
        if button_pressed(buttons_low, 7):
            steering = -1
        elif button_pressed(buttons_low, 5):
            steering = 1

    if button_pressed(buttons_high, 6):  # X: toc do nhanh
        speed_limit = FAST_SPEED
    elif button_pressed(buttons_high, 5):  # O: toc do cham
        speed_limit = SLOW_SPEED
    else:
        speed_limit = NORMAL_SPEED

    robot_move(throttle, steering, speed_limit)

    # D-pad UP: nang; DOWN: ha. Nhan dong thoi thi bo qua.
    lift_up = button_pressed(buttons_low, 4)
    lift_down = button_pressed(buttons_low, 6)
    if lift_up != lift_down:
        servo_lift.set_angle(LIFT_UP_ANGLE if lift_up else LIFT_DOWN_ANGLE)

    # L1: gap; L2: nha.
    grip_close = button_pressed(buttons_high, 2)
    grip_open = button_pressed(buttons_high, 0)
    if grip_close != grip_open:
        servo_grip.set_angle(GRIP_CLOSE_ANGLE if grip_close else GRIP_OPEN_ANGLE)


def main():
    print("ESP32-S3 robot: dang cau hinh tay cam PS2...")
    stop_drive()
    ps2_config()
    print("San sang. L1 gap, L2 nha, R1 nang, R2 ha.")

    try:
        while True:
            control_once()
            sleep(CONTROL_PERIOD)
    finally:
        stop_drive()


if __name__ == "__main__":
    if HARDWARE_AVAILABLE:
        main()
    else:
        print("ESP32 hardware unavailable. Run this program with MicroPython on the ESP32-S3.")