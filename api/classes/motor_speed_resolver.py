from . import constants
from .motor_speed_calculator import MotorSpeedCalculator


class MotorSpeedResolver:
    def _resolve_forward_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.increase_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.decrease_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_more_forward_left()
        else:
            self._motor_speed_calculator.move_more_forward_right()

    def _resolve_forward_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.decrease_forward_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_more_forward_left()
        else:
            self._motor_speed_calculator.move_more_forward_right()

    _resolve_forward_left_direction = _resolve_forward_right_direction

    def _resolve_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_less_inplace_right()
        else:
            self._motor_speed_calculator.move_more_inplace_right()

    def _resolve_left_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.equalise_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_more_inplace_left()
        else:
            self._motor_speed_calculator.move_less_inplace_left()

    def _resolve_reverse_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.decrease_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.increase_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_more_reverse_left()
        else:
            self._motor_speed_calculator.move_more_reverse_right()

    def _resolve_reverse_right_direction(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.decrease_reverse_speed()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.equalise_speed('min')
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_more_reverse_left()
        else:
            self._motor_speed_calculator.move_more_reverse_right()

    _resolve_reverse_left_direction = _resolve_reverse_right_direction

    def _resolve_from_stopped_state(self):
        if self._target_action == constants.TARGET_ACTION_FORWARD:
            self._motor_speed_calculator.move_forward()
        elif self._target_action == constants.TARGET_ACTION_REVERSE:
            self._motor_speed_calculator.reverse()
        elif self._target_action == constants.TARGET_ACTION_LEFT:
            self._motor_speed_calculator.move_inplace_left()
        else:
            self._motor_speed_calculator.move_inplace_right()

    def resolve(self, state, target_action):
        self._target_action = target_action
        current_direction = state['current_direction']
        self._motor_speed_calculator = MotorSpeedCalculator(state)

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

        return self._motor_speed_calculator.speeds()
