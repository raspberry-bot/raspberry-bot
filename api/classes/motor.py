import os
from api.utils.clip import clip

import importlib.util
try:
    importlib.util.find_spec('RPi.GPIO')
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    import FakeRPi.GPIO as GPIO


class Motor:
    def __init__(self, pins, frequency=20):
        self.frequency = frequency
        GPIO.setmode(GPIO.BCM)

        self.forward_pin = pins['forward']
        self.reverse_pin = pins['reverse']
        self.control_pin = pins['control']

        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.reverse_pin, GPIO.OUT)
        GPIO.setup(self.control_pin, GPIO.OUT)

        self._speed_control = GPIO.PWM(self.control_pin, frequency)
        self._speed_control.start(0)

    def _set_speed(self, speed):
        speed = clip(abs(speed))
        self._speed_control.ChangeDutyCycle(speed)
        return speed

    def _set_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.reverse_pin, GPIO.LOW)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.reverse_pin, GPIO.HIGH)

    def _forward(self, speed_percent):
        self._set_direction('forward')
        return self._set_speed(speed_percent)

    def _reverse(self, speed_percent):
        self._set_direction('reverse')
        return self._set_speed(speed_percent)

    def _stop(self):
        return self._set_speed(0)

    def move(self, speed_percent):
        if speed_percent == 0:
            speed = self._stop()
        elif speed_percent < 0:
            speed = -1 * self._reverse(speed_percent)
        else:
            speed = self._forward(speed_percent)

        return speed
