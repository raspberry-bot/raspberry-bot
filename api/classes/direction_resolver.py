from . import constants


class DirectionResolver:
    def __init__(self):
        self._ml_speed = 0
        self._mr_speed = 0

    def _resolve_forward_direction(self):
        diff = self._ml_speed - self._mr_speed
        if diff == 0:
            self.direction = constants.DIRECTION_FORWARD
        elif diff < 0:
            self.direction = constants.DIRECTION_FORWARD_LEFT
        else:
            self.direction = constants.DIRECTION_FORWARD_RIGHT

    def _resolve_reverse_direction(self):
        diff = self._ml_speed - self._mr_speed
        if diff == 0:
            self.direction = constants.DIRECTION_REVERSE
        elif diff > 0:
            self.direction = constants.DIRECTION_REVERSE_LEFT
        else:
            self.direction = constants.DIRECTION_REVERSE_RIGHT

    def _resolve_in_place_direction(self):
        if self._ml_speed < 0:
            self.direction = constants.DIRECTION_LEFT
        else:
            self.direction = constants.DIRECTION_RIGHT

    def _is_forward_direction(self):
        return (self._ml_speed > 0 and self._mr_speed > 0) or \
            (self._ml_speed > 0 and self._mr_speed == 0) or \
            (self._ml_speed == 0 and self._mr_speed > 0)

    def _is_reverse_direction(self):
        return (self._ml_speed < 0 and self._mr_speed < 0) or \
            (self._ml_speed < 0 and self._mr_speed == 0) or \
            (self._ml_speed == 0 and self._mr_speed < 0)

    def resolve(self, speeds):
        self._ml_speed = speeds['left_motor_speed']
        self._mr_speed = speeds['right_motor_speed']

        if self._ml_speed == 0 and self._mr_speed == 0:
            self.direction = constants.DIRECTION_NONE
            return self.direction

        if self._is_forward_direction():
            self._resolve_forward_direction()
        elif self._is_reverse_direction():
            self._resolve_reverse_direction()
        else:
            self._resolve_in_place_direction()

        return self.direction
