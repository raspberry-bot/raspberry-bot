from . import constants
from .motor import Motor
from .direction_resolver import DirectionResolver
from .speed_resolver import SpeedResolver


class Driver:
    def __init__(self, left_motor_pins, right_motor_pins):
        # Assign pins to motors.
        self._left_motor = Motor(left_motor_pins)
        self._right_motor = Motor(right_motor_pins)

        self.direction_resolver = DirectionResolver()
        self.speed_resolver = SpeedResolver()

        self.stop()

    def _current_direction(self):
        left_motor_speed = self._state['left_motor_speed']
        right_motor_speed = self._state['right_motor_speed']

        return self.direction_resolver.resolve(left_motor_speed, right_motor_speed)

    def _set_speed(self, target_action):
        speeds = self.speed_resolver.resolve(self._state, target_action)

        left_motor_speed = speeds['left_motor_speed']
        right_motor_speed = speeds['right_motor_speed']

        self._set_state(left_motor_speed, right_motor_speed)

        self._left_motor.move(left_motor_speed)
        self._right_motor.move(right_motor_speed)

    def _set_state(self, left_motor_speed, right_motor_speed):
        self._state = {
            'left_motor_speed': left_motor_speed,
            'right_motor_speed': right_motor_speed
        }

        self._state['current_direction'] = self._current_direction()

        return self._state

    def stop(self):
        self._left_motor.move(0)
        self._right_motor.move(0)

        return self._set_state(0, 0)

    def forward(self):
        self._set_speed(constants.TARGET_ACTION_FORWARD)

        return self._state

    def reverse(self):
        self._set_speed(constants.TARGET_ACTION_REVERSE)

        return self._state

    def left(self):
        self._set_speed(constants.TARGET_ACTION_LEFT)

        return self._state

    def right(self):
        self._set_speed(constants.TARGET_ACTION_RIGHT)

        return self._state
