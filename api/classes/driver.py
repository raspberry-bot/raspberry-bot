from .motor import Motor

class Driver:
    def __init__(self, left_motor_pins, right_motor_pins):
        # Assign pins to motors.
        self._left_motor = Motor(left_motor_pins)
        self._right_motor = Motor(right_motor_pins)
        self._state = {
            'state': 'stopped',
            'left_speed': 0,
            'right_speed': 0
        }
        self._SPEED_INCREMENT = 10

    def _current_direction(self):
        if self._state['left_speed'] > self._state['right_speed']:
            self._state['state'] = 'right'
            return 'right'
        elif self._state['left_speed'] < self._state['right_speed']:
            self._state['state'] = 'left'
            return 'left'

        return self._state['state']

    def stop(self):
        self._left_motor.stop()
        self._right_motor.stop()

        self._state = {
            'state': 'stopped',
            'left_speed': 0,
            'right_speed': 0
        }

        return self._state

    def forward(self):
        if self._current_direction() in ['forward', 'stopped']:
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() == 'reverse':
            self.stop()
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() in ['right', 'left']:
            target_speed = min(
                self._state['left_speed'], self._state['right_speed'])
            self._state['left_speed'] = target_speed
            self._state['right_speed'] = target_speed

        left_speed = self._left_motor.forward(self._state['left_speed'])
        right_speed = self._right_motor.forward(self._state['left_speed'])

        self._state = {
            'state': 'forward',
            'left_speed': left_speed,
            'right_speed': right_speed
        }

        return self._state

    def reverse(self):
        if self._current_direction() in ['reverse', 'stopped']:
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() == 'forward':
            self.stop()
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() in ['right', 'left']:
            target_speed = min(
                self._state['left_speed'], self._state['right_speed'])
            self._state['left_speed'] = target_speed
            self._state['right_speed'] = target_speed

        left_speed = self._left_motor.reverse(self._state['left_speed'])
        right_speed = self._right_motor.reverse(self._state['left_speed'])

        self._state = {
            'state': self._current_direction(),
            'left_speed': left_speed,
            'right_speed': right_speed
        }
        return self._state

    def left(self):
        if self._current_direction() == 'stopped':
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
            left_speed = self._left_motor.forward(self._state['left_speed'])
            right_speed = self._right_motor.reverse(self._state['right_speed'])

            self._state = {
                'state': 'left',
                'left_speed': left_speed,
                'right_speed': right_speed
            }
            return self._state

        if self._current_direction() == 'forward':
            self._state['left_speed'] -= self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() == 'reverse':
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] -= self._SPEED_INCREMENT
        elif self._current_direction() in ['left', 'right']:
            self._state['left_speed'] -= self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT

        left_speed = self._left_motor.forward(self._state['left_speed'])
        right_speed = self._right_motor.forward(self._state['right_speed'])

        self._state = {
            'state': self._current_direction(),
            'left_speed': left_speed,
            'right_speed': right_speed
        }
        return self._state

    def right(self):
        if self._current_direction() == 'stopped':
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
            left_speed = self._left_motor.forward(self._state['left_speed'])
            right_speed = self._right_motor.reverse(self._state['right_speed'])

            self._state = {
                'state': self._current_direction(),
                'left_speed': left_speed,
                'right_speed': right_speed
            }
            return self._state

        if self._current_direction() == 'forward':
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] -= self._SPEED_INCREMENT
        elif self._current_direction() == 'reverse':
            self._state['left_speed'] -= self._SPEED_INCREMENT
            self._state['right_speed'] += self._SPEED_INCREMENT
        elif self._current_direction() in ['left', 'right']:
            self._state['left_speed'] += self._SPEED_INCREMENT
            self._state['right_speed'] -= self._SPEED_INCREMENT

        left_speed = self._left_motor.forward(self._state['left_speed'])
        right_speed = self._right_motor.forward(self._state['right_speed'])

        self._state = {
            'state': self._current_direction(),
            'left_speed': left_speed,
            'right_speed': right_speed
        }

        return self._state

    # def _velocity_received_callback(self, message):
    #     """Handle new velocity command message."""

    #     self._last_received = rospy.get_time()

    #     # Extract linear and angular velocities from the message
    #     linear = message.linear.x
    #     angular = message.angular.z

    #     # Calculate wheel speeds in m/s
    #     left_speed = linear - angular*self._wheel_base/2
    #     right_speed = linear + angular*self._wheel_base/2

    #     # Ideally we'd now use the desired wheel speeds along
    #     # with data from wheel speed sensors to come up with the
    #     # power we need to apply to the wheels, but we don't have
    #     # wheel speed sensors. Instead, we'll simply convert m/s
    #     # into percent of maximum wheel speed, which gives us a
    #     # duty cycle that we can apply to each motor.
    #     self._left_speed_percent = (100 * left_speed/self._max_speed)
    #     self._right_speed_percent = (100 * right_speed/self._max_speed)

    # def run(self):
    #     """The control loop of the driver."""

    #     rate = rospy.Rate(self._rate)

    #     while not rospy.is_shutdown():
    #         # If we haven't received new commands for a while, we
    #         # may have lost contact with the commander-- stop
    #         # moving
    #         delay = rospy.get_time() - self._last_received
    #         if delay < self._timeout:
    #             self._left_motor.move(self._left_speed_percent)
    #             self._right_motor.move(self._right_speed_percent)
    #         else:
    #             self._left_motor.move(0)
    #             self._right_motor.move(0)

    #         rate.sleep()
