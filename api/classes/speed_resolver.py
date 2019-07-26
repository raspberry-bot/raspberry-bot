from . import constants


class ForwardMotorCommands:
    @staticmethod
    def increase_speed(speed):
        speed += constants.SPEED_INCREMENT
        return speed

    @staticmethod
    def decrease_speed(speed):
        speed -= constants.SPEED_INCREMENT
        return speed


class ReverseMotorCommands:
    @staticmethod
    def increase_speed(speed):
        speed -= constants.SPEED_INCREMENT
        return speed

    @staticmethod
    def decrease_speed(speed):
        speed += constants.SPEED_INCREMENT
        return speed


class SpeedResolver:
    def __init__(self):
        self._speed_increment = constants.SPEED_INCREMENT

    def _equalise_speed(self, operation='max'):
        if operation == 'max':
            speed = max(self.left_motor_speed, self.right_motor_speed)
        else:
            speed = min(self.left_motor_speed, self.right_motor_speed)

        self.left_motor_speed = self.right_motor_speed = speed

    # motor commands
    def _forward_increase_right_motor(self):
        self.right_motor_speed = ForwardMotorCommands.increase_speed(self.right_motor_speed)

    def _forward_decrease_right_motor(self):
        self.right_motor_speed = ForwardMotorCommands.decrease_speed(self.right_motor_speed)

    def _forward_increase_left_motor(self):
        self.left_motor_speed = ForwardMotorCommands.increase_speed(self.left_motor_speed)

    def _forward_decrease_left_motor(self):
        self.left_motor_speed = ForwardMotorCommands.decrease_speed(self.left_motor_speed)

    def _reverse_increase_right_motor(self):
        self.right_motor_speed = ReverseMotorCommands.increase_speed(self.right_motor_speed)

    def _reverse_decrease_right_motor(self):
        self.right_motor_speed = ReverseMotorCommands.decrease_speed(self.right_motor_speed)

    def _reverse_increase_left_motor(self):
        self.left_motor_speed = ReverseMotorCommands.increase_speed(self.left_motor_speed)

    def _reverse_decrease_left_motor(self):
        self.left_motor_speed = ReverseMotorCommands.decrease_speed(self.left_motor_speed)

    # relative drive commands
    def _increase_forward_speed(self):
        self._forward_increase_left_motor()
        self._forward_increase_right_motor()

    def _decrease_forward_speed(self):
        self._forward_decrease_left_motor()
        self._forward_decrease_right_motor()

    def _increase_reverse_speed(self):
        self._reverse_increase_left_motor()
        self._reverse_increase_right_motor()

    def _decrease_reverse_speed(self):
        self._reverse_decrease_left_motor()
        self._reverse_decrease_right_motor()

    def _move_more_forward_right(self):
        self._forward_increase_left_motor()
        self._forward_decrease_right_motor()

    def _move_more_forward_left(self):
        self._forward_decrease_left_motor()
        self._forward_increase_right_motor()

    def _move_more_reverse_left(self):
        self._reverse_decrease_left_motor()
        self._reverse_increase_right_motor()

    def _move_more_reverse_right(self):
        self._reverse_increase_left_motor()
        self._reverse_decrease_right_motor()

    def _move_more_in_place_left(self):
        self.left_motor_speed -= self._speed_increment
        self.right_motor_speed += self._speed_increment

    _move_less_in_place_right = _move_more_in_place_left

    def _move_more_in_place_right(self):
        self.left_motor_speed += self._speed_increment
        self.right_motor_speed -= self._speed_increment

    _move_less_in_place_left = _move_more_in_place_right

    # absolute drive commands
    def _reverse(self):
        self.left_motor_speed = self.right_motor_speed = (-1 * self._speed_increment)

    def _move_forward(self):
        self.left_motor_speed = self.right_motor_speed = self._speed_increment

    def _move_forward_right(self):
        self.left_motor_speed = self._speed_increment
        self.right_motor_speed = 0

    def _move_forward_left(self):
        self.left_motor_speed = 0
        self.right_motor_speed = self._speed_increment

    def _move_inplace_left(self):
        self.left_motor_speed = -1 * self._speed_increment
        self.right_motor_speed = self._speed_increment

    def _move_inplace_right(self):
        self.left_motor_speed = self._speed_increment
        self.right_motor_speed = -1 * self._speed_increment

    def _resolve_forward_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._increase_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._decrease_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_more_forward_left()
        else:
            self._move_more_forward_right()

    def _resolve_forward_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._decrease_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_more_forward_left()
        else:
            self._move_more_forward_right()

    _resolve_forward_left_direction = _resolve_forward_right_direction

    def _resolve_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_less_in_place_right()
        else:
            self._move_more_in_place_right()

    def _resolve_left_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_more_in_place_left()
        else:
            self._move_less_in_place_left()

    def _resolve_reverse_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._decrease_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._increase_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_more_reverse_left()
        else:
            self._move_more_reverse_right()

    def _resolve_reverse_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._decrease_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_more_reverse_left()
        else:
            self._move_more_reverse_right()

    _resolve_reverse_left_direction = _resolve_reverse_right_direction

    def _resolve_from_stopped_state(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._move_forward()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._reverse()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._move_inplace_left()
        else:
            self._move_inplace_right()

    def resolve(self, state, target_action):
        self._target_action = target_action
        current_direction = state['current_direction']
        self.left_motor_speed = state['left_motor_speed']
        self.right_motor_speed = state['right_motor_speed']

        if current_direction == constants.DIRECTION_FORWARD:
            self._resolve_forward_direction()
        elif current_direction == constants.DIRECTION_FORWARD_RIGHT:
            self._resolve_forward_right_direction()
        elif current_direction == constants.DIRECTION_FORWARD_LEFT:
            self._resolve_forward_left_direction()
        elif current_direction == constants.DIRECTION_REVERSE:
            self._resolve_reverse_direction()
        elif current_direction == constants.DIRECTION_REVERSE_RIGHT:
            self._resolve_reverse_right_direction()
        elif current_direction == constants.DIRECTION_REVERSE_LEFT:
            self._resolve_reverse_left_direction()
        elif current_direction == constants.DIRECTION_RIGHT:
            self._resolve_right_direction()
        elif current_direction == constants.DIRECTION_LEFT:
            self._resolve_left_direction()
        else:
            self._resolve_from_stopped_state()

        return {
            'left_motor_speed': self.left_motor_speed,
            'right_motor_speed': self.right_motor_speed
        }
