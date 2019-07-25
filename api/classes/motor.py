import RPi.GPIO as GPIO


def _clip(value, minimum=0, maximum=100):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


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
        self.speed = 0

    def _set_speed(self, speed):
        speed = _clip(abs(speed))
        self._speed_control.start(speed)
        return speed

    def _set_direction_and_speed(self, direction, speed_percent):
        if direction == 'forward':
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.reverse_pin, GPIO.LOW)
            polarity = 1
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.reverse_pin, GPIO.HIGH)
            polarity = -1

        self.speed = (polarity * self._set_speed(speed_percent))

    def _forward(self, speed_percent):
        self._set_direction_and_speed('forward', speed_percent)

    def _reverse(self, speed_percent):
        self._set_direction_and_speed('reverse', speed_percent)

    def _stop(self):
        return self._set_speed(0)

    def move(self, speed_percent):
        if speed_percent == 0:
            self._stop()
        elif speed_percent < 0:
            self._reverse(speed_percent)
        else:
            self._forward(speed_percent)

        return self.speed
