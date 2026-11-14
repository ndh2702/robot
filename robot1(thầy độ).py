from machine import Pin, PWM
from time import sleep, sleep_us

# ==========================================
# 1. CAU HINH TAY CAM PS2
# ==========================================

PS2_DAT = Pin(12, Pin.IN, Pin.PULL_UP)
PS2_CMD = Pin(13, Pin.OUT, value=1)
PS2_CLK = Pin(14, Pin.OUT, value=1)
PS2_ATT = Pin(15, Pin.OUT, value=1)


def ps2_transfer(data):
    received = 0

    for i in range(8):
        PS2_CLK.value(1)
        PS2_CMD.value((data >> i) & 1)
        sleep_us(10)

        PS2_CLK.value(0)
        sleep_us(10)

        received |= PS2_DAT.value() << i

    PS2_CLK.value(1)
    return received


def ps2_read():
    PS2_ATT.value(0)
    sleep_us(20)

    ps2_transfer(0x01)
    mode = ps2_transfer(0x42)
    ps2_transfer(0x00)

    data = []
    for i in range(6):
        data.append(ps2_transfer(0x00))

    PS2_ATT.value(1)

    if mode not in (0x41, 0x73, 0x79):
        return None

    # O che do digital khong co gia tri can analog.
    if mode == 0x41:
        data[2] = 128
        data[3] = 128
        data[4] = 128
        data[5] = 128

    return data


def ps2_config():
    PS2_ATT.value(0)

    ps2_transfer(0x01)
    ps2_transfer(0x43)
    ps2_transfer(0x00)

    for value in [0x01, 0x00, 0x00, 0x00, 0x00]:
        ps2_transfer(value)

    PS2_ATT.value(1)
    sleep(0.01)

    PS2_ATT.value(0)

    ps2_transfer(0x01)
    ps2_transfer(0x44)
    ps2_transfer(0x00)
    ps2_transfer(0x01)
    ps2_transfer(0x03)

    for i in range(3):
        ps2_transfer(0x00)

    PS2_ATT.value(1)
    sleep(0.01)

    PS2_ATT.value(0)

    ps2_transfer(0x01)
    ps2_transfer(0x4F)
    ps2_transfer(0x00)
    ps2_transfer(0xFF)
    ps2_transfer(0xFF)
    ps2_transfer(0x03)
    ps2_transfer(0x00)
    ps2_transfer(0x00)

    PS2_ATT.value(1)
    sleep(0.01)

    PS2_ATT.value(0)

    ps2_transfer(0x01)
    ps2_transfer(0x43)
    ps2_transfer(0x00)
    ps2_transfer(0x00)
    ps2_transfer(0x00)
    ps2_transfer(0x00)
    ps2_transfer(0x00)
    ps2_transfer(0x00)

    PS2_ATT.value(1)
    sleep(0.01)


# ==========================================
# 2. CAU HINH 4 DONG CO
# ==========================================

# Moi dong co dung 2 chan dieu khien
# PWM tren IN1 hoac IN2 de dieu chinh toc do

class Motor:

    def __init__(self, pin1, pin2, reverse=False):

        self.pwm1 = PWM(Pin(pin1), freq=1000)
        self.pwm2 = PWM(Pin(pin2), freq=1000)

        self.reverse = reverse
        self.stop()

    def run(self, speed):

        speed = max(-100, min(100, speed))

        if self.reverse:
            speed = -speed

        duty = int(abs(speed) * 65535 / 100)

        if speed > 0:
            self.pwm1.duty_u16(duty)
            self.pwm2.duty_u16(0)

        elif speed < 0:
            self.pwm1.duty_u16(0)
            self.pwm2.duty_u16(duty)

        else:
            self.stop()

    def stop(self):
        self.pwm1.duty_u16(0)
        self.pwm2.duty_u16(0)


class ServoSG90:

    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50)
        self.current_angle = None

    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        if angle == self.current_angle:
            return

        pulse_us = 500 + (angle * 2000 / 180)
        duty = int(pulse_us * 65535 / 20000)
        self.pwm.duty_u16(duty)
        self.current_angle = angle


# ==========================================
# 3. KHOI TAO 4 DONG CO
# ==========================================

motor_FL = Motor(4, 5, False)   # Truoc trai
motor_FR = Motor(6, 7, True)    # Truoc phai
motor_RL = Motor(8, 9, False)   # Sau trai
motor_RR = Motor(10, 11, True) # Sau phai
servo_lift = ServoSG90(16)
servo_gripper = ServoSG90(17)

LIFT_UP_ANGLE = 30
LIFT_DOWN_ANGLE = 90
GRIP_CLOSE_ANGLE = 30
GRIP_OPEN_ANGLE = 90


def stop_all():
    motor_FL.stop()
    motor_FR.stop()
    motor_RL.stop()
    motor_RR.stop()


# ==========================================
# 4. DIEU KHIEN ROBOT
# ==========================================

MAX_SPEED = 70
DEAD_ZONE = 15
CONTROL_PERIOD = 0.01


def deadzone(value):
    if abs(value) < DEAD_ZONE / 128:
        return 0
    return value


def joystick_value(value):
    return (128 - value) / 128


def robot_move(throttle, steering):

    # Phoi hop tien lui va re trai phai
    left = throttle + steering
    right = throttle - steering

    # Chuan hoa toc do
    maximum = max(1, abs(left), abs(right))

    left = left / maximum * MAX_SPEED
    right = right / maximum * MAX_SPEED

    motor_FL.run(left)
    motor_RL.run(left)

    motor_FR.run(right)
    motor_RR.run(right)


# ==========================================
# 5. CHUONG TRINH CHINH
# ==========================================

print("ROBOT ESP32-S3 STARTING...")

stop_all()
sleep(1)

print("DANG KHOI TAO TAY CAM PS2")

ps2_config()

print("SAN SANG DIEU KHIEN ROBOT")


while True:

    try:

        data = ps2_read()

        # Mat ket noi hoac khong doc duoc tay cam
        if data is None:
            stop_all()
            sleep(0.1)
            continue

        # Sau byte 0x5A, goi PS2 lan luot la nut thap, nut cao,
        # analog phai X/Y, analog trai X/Y.
        buttons_low = data[0]
        buttons_high = data[1]

        left_x = data[4]
        left_y = data[5]

        # Nut START: Dung khan cap
        if not (buttons_low & (1 << 3)):
            stop_all()
            sleep(0.05)
            continue

        # Dieu khien bang can analog trai
        throttle = joystick_value(left_y)
        steering = (left_x - 128) / 128

        throttle = deadzone(throttle)
        steering = deadzone(steering)

        # Dieu khien bang phim mui ten neu can analog o giua
        if abs(throttle) < 0.01 and abs(steering) < 0.01:

            if not (buttons_low & (1 << 4)):
                throttle = 1

            elif not (buttons_low & (1 << 6)):
                throttle = -1

            if not (buttons_low & (1 << 7)):
                steering = -1

            elif not (buttons_low & (1 << 5)):
                steering = 1

        # Nut X: Tang toc
        if not (buttons_high & (1 << 6)):
            MAX_SPEED = 100

        # Nut O: Giam toc
        elif not (buttons_high & (1 << 5)):
            MAX_SPEED = 40

        else:
            MAX_SPEED = 70

        robot_move(throttle, steering)

        # R1/R2 dieu khien nang-ha; L1/L2 dieu khien gap-nha.
        lift_up = not (buttons_high & (1 << 3))
        lift_down = not (buttons_high & (1 << 1))
        grip_close = not (buttons_high & (1 << 2))
        grip_open = not (buttons_high & (1 << 0))

        if lift_up != lift_down:
            servo_lift.set_angle(
                LIFT_UP_ANGLE if lift_up else LIFT_DOWN_ANGLE
            )

        if grip_close != grip_open:
            servo_gripper.set_angle(
                GRIP_CLOSE_ANGLE if grip_close else GRIP_OPEN_ANGLE
            )

        sleep(CONTROL_PERIOD)

    except Exception as error:

        print("LOI:", error)

        stop_all()
        sleep(0.2)