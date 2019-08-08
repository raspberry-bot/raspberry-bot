from . import constants
from api.utils.clip import clip
from .direction_resolver import DirectionResolver
from .motor_speed_resolver import MotorSpeedResolver
from .motor import Motor
import math

LEFT_MOTOR_PINS = {
    'forward': 3,
    'reverse': 4,
    'control': 2
}

RIGHT_MOTOR_PINS = {
    'forward': 20,
    'reverse': 16,
    'control': 21
}

class Driver:
    def __init__(
        self,
        left_motor_pins=LEFT_MOTOR_PINS, right_motor_pins=RIGHT_MOTOR_PINS,
        min_speed=-100, max_speed=100,
        min_cmd=-50, max_cmd=50
    ):
        # Assign pins to motors.
        self.left_motor = Motor(left_motor_pins)
        self.right_motor = Motor(right_motor_pins)
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_cmd = min_cmd
        self.max_cmd = max_cmd

    def command_to_diff(self, x, y, min_speed, max_speed):
        # If x and y are 0, then there is not much to calculate...
        if x == 0 and y == 0:
            return (0, 0)
        # First Compute the angle in deg
        # First hypotenuse
        z = math.sqrt(x * x + y * y)
        # angle in radians
        rad = math.acos(math.fabs(x) / z)
        # and in degrees
        angle = rad * 180 / math.pi

        # Now angle indicates the measure of turn
        # Along a straight line, with an angle o, the turn co-efficient is same
        # this applies for angles between 0-90, with angle 0 the coeff is -1
        # with angle 45, the co-efficient is 0 and with angle 90, it is 1
        tcoeff = -1 + (angle / 90) * 2
        turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
        turn = round(turn * 100, 0) / 100
        # And max of y or x is the movement
        mov = max(math.fabs(y), math.fabs(x))
        # First and third quadrant
        if (x >= 0 and y >= 0) or (x < 0 and y < 0):
            raw_left = mov
            raw_right = turn
        else:
            raw_right = mov
            raw_left = turn
        # Reverse polarity
        if y < 0:
            raw_left = 0 - raw_left
            raw_right = 0 - raw_right

        # Map the values onto the defined ranges
        left_out = self.map(raw_left, min_speed, max_speed)
        right_out = self.map(raw_right, min_speed, max_speed)
        return (left_out, right_out)

    def map(self, v, min_speed, max_speed):
        # Check that the value is at least in_min
        if v < self.min_cmd:
            v = self.min_cmd
        # Check that the value is at most in_max
        if v > self.max_cmd:
            v = self.max_cmd
        return (v - self.min_cmd) * (max_speed - min_speed) // (self.max_cmd - self.min_cmd) + min_speed
