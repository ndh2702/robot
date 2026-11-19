"""ESP32 robot: receive PS2 commands over ESP-NOW and drive a mecanum base.

Run this file on the robot's central ESP32 using MicroPython.
"""

import struct
import time

import espnow  # pyright: ignore[reportMissingModuleSource]
import network  # pyright: ignore[reportMissingModuleSource]
from machine import Pin, PWM  # pyright: ignore[reportMissingModuleSource]


# Copy the handheld ESP32's MAC address here after reading its serial output.
CONTROLLER_MAC = "AA:BB:CC:DD:EE:FF"
PACKET_MAGIC = 0xA7
PACKET_FORMAT = "<BhhhH"
MAX_SPEED = 1023
COMMAND_TIMEOUT_MS = 500

# Preserve the wheel direction pins from robot1.py. Each TB6612 channel also
# needs its own PWM pin. STBY on both driver boards must be tied to 3V3.
WHEELS = (
    (16, 17, 32),  # Front-left: IN1, IN2, PWM
    (18, 19, 33),  # Front-right
    (25, 26, 23),  # Rear-left
    (27, 14, 4),   # Rear-right
)
SERVO_PIN = 13
LIGHT_PIN = 12  # Use a MOSFET/driver; do not power a flashlight from this GPIO.

SERVO_MIN_ANGLE = 10
SERVO_MAX_ANGLE = 170
SERVO_START_ANGLE = 90
SERVO_STEP_DEGREES = 2
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500

# Command bits sent by esp-32.py.
BUTTON_LIGHT = 1 << 0
BUTTON_SERVO_UP = 1 << 1
BUTTON_SERVO_DOWN = 1 << 2
BUTTON_SERVO_CENTER = 1 << 3


def parse_mac(mac_text):
    parts = mac_text.split(":")
    if len(parts) != 6:
        raise ValueError("MAC phải có dạng AA:BB:CC:DD:EE:FF")
    return bytes(int(part, 16) for part in parts)


def format_mac(mac):
    return ":".join("%02X" % byte for byte in mac)


def station_interface():
    interface_id = (
        network.STA_IF
        if hasattr(network, "STA_IF")
        else network.WLAN.IF_STA
    )
    wlan = network.WLAN(interface_id)
    wlan.active(True)
    return wlan


class TB6612Motor:
    """One TB6612 channel, with separate direction and PWM pins."""

    def __init__(self, in1_pin, in2_pin, pwm_pin):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin), freq=1000)
        self.stop()

    def set_speed(self, speed):
        speed = max(-MAX_SPEED, min(MAX_SPEED, int(speed)))
        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
            self.pwm.duty(speed)
        elif speed < 0:
            self.in1.value(0)
            self.in2.value(1)
            self.pwm.duty(-speed)
        else:
            self.stop()

    def stop(self):
        self.pwm.duty(0)
        self.in1.value(0)
        self.in2.value(0)


class Servo:
    def __init__(self, pin_num):
        self.pwm = PWM(Pin(pin_num), freq=50)
        self.angle = SERVO_START_ANGLE
        self.set_angle(self.angle)

    def set_angle(self, angle):
        self.angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, int(angle)))
        pulse_us = SERVO_MIN_PULSE_US + (
            self.angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) // 180
        )
        self.pwm.duty(pulse_us * 1023 // 20000)


class Robot:
    def __init__(self):
        self.wheels = tuple(TB6612Motor(*pins) for pins in WHEELS)
        self.servo = Servo(SERVO_PIN)
        self.light = Pin(LIGHT_PIN, Pin.OUT)
        self.light.value(0)
        self.stop()

    def drive(self, forward, strafe, turn):
        # Wheel order and signs match the mecanum mix used by robot1.py.
        speeds = (
            forward - strafe - turn,
            forward + strafe + turn,
            forward + strafe - turn,
            forward - strafe + turn,
        )
        peak = max(abs(speed) for speed in speeds)
        if peak > MAX_SPEED:
            speeds = tuple(speed * MAX_SPEED // peak for speed in speeds)
        for wheel, speed in zip(self.wheels, speeds):
            wheel.set_speed(speed)

    def stop(self):
        for wheel in self.wheels:
            wheel.stop()


def apply_buttons(robot, buttons, previous_buttons):
    if buttons & BUTTON_LIGHT and not previous_buttons & BUTTON_LIGHT:
        robot.light.value(not robot.light.value())

    if buttons & BUTTON_SERVO_CENTER and not previous_buttons & BUTTON_SERVO_CENTER:
        robot.servo.set_angle(SERVO_START_ANGLE)
    elif buttons & BUTTON_SERVO_UP:
        robot.servo.set_angle(robot.servo.angle + SERVO_STEP_DEGREES)
    elif buttons & BUTTON_SERVO_DOWN:
        robot.servo.set_angle(robot.servo.angle - SERVO_STEP_DEGREES)


def run():
    wlan = station_interface()
    print("MAC robot:", format_mac(wlan.config("mac")))

    if CONTROLLER_MAC == "AA:BB:CC:DD:EE:FF":
        raise RuntimeError(
            "Hãy thay CONTROLLER_MAC bằng MAC của ESP32 tay cầm trước khi chạy."
        )

    controller_mac = parse_mac(CONTROLLER_MAC)
    radio = espnow.ESPNow()
    radio.active(True)
    robot = Robot()
    previous_buttons = 0
    last_command_ms = None

    print("Đang chờ lệnh từ tay cầm PS2...")

    while True:
        sender, message = radio.recv(20)
        now = time.ticks_ms()

        if sender == controller_mac and message is not None:
            if len(message) == struct.calcsize(PACKET_FORMAT):
                magic, forward, strafe, turn, buttons = struct.unpack(
                    PACKET_FORMAT, message
                )
                if magic == PACKET_MAGIC:
                    forward = max(-MAX_SPEED, min(MAX_SPEED, forward))
                    strafe = max(-MAX_SPEED, min(MAX_SPEED, strafe))
                    turn = max(-MAX_SPEED, min(MAX_SPEED, turn))
                    robot.drive(forward, strafe, turn)
                    apply_buttons(robot, buttons, previous_buttons)
                    previous_buttons = buttons
                    last_command_ms = now

        if (
            last_command_ms is not None
            and time.ticks_diff(now, last_command_ms) > COMMAND_TIMEOUT_MS
        ):
            robot.stop()
            previous_buttons = 0
            last_command_ms = None


if __name__ == "__main__":
    run()
