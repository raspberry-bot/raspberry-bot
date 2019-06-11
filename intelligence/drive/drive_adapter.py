import sys
import time
import RPi.GPIO as GPIO
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist


class Drive:

    """
    catkin_create_pkg drive std_msgs rospy roscpp sensor_msgs geometry_msgs cv_bridge
    """

    def __init__(self):
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.ros_adapter)
        self.mode = GPIO.getmode()

        self.IN1 = 16
        self.IN2 = 36
        self.IN3 = 18
        self.IN4 = 38

        self.control_a = 22
        self.control_b = 40

        self._setup_gpio()

        self.pwm_a = GPIO.PWM(self.control_a, 100)
        self.pwm_b = GPIO.PWM(self.control_b, 100)

        self.duty_cycle_increased = False

        self._setup_motors()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        GPIO.setup(self.control_a, GPIO.OUT)
        GPIO.setup(self.control_b, GPIO.OUT)

    def _setup_motors(self):
        if self.duty_cycle_increased is False:
            for pct in range(100):
                self.pwm_a.start(pct)
                self.pwm_b.start(pct)
                time.sleep(1.0/1000)
            self.duty_cycle_increased = True

    def shutdown(self):
        print('Shutting down the motors...')
        print('Decreasing ChangeDutyCycle to 0...')
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        print('Clean up...')
        GPIO.cleanup()
        sys.exit(0)

    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        print('Decreasing ChangeDutyCycle to 0...')
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.duty_cycle_increased = False

    def forward(self):
        print("Moving forward...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def reverse(self):
        print("Moving reverse...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def right(self):
        print("Moving right...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        # GPIO.output(self.IN4, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def left(self):
        print("Moving left...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.LOW)
        # GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def ros_adapter(self, data):
        speed = data.linear.x
        steer = data.angular.z
        if steer == 0 and speed == 0:
            self.stop()
        elif steer < 0:
            self.right()
        elif steer > 0:
            self.left()
        elif steer == 0 and speed > 0:
            self.forward()
        elif speed < 0:
            self.reverse()


if __name__ == '__main__':
    try:
        rospy.init_node('Custom_Drive_Adapter', anonymous=True)
        drive = Drive()
        rospy.spin()
    except KeyboardInterrupt:
        drive.shutdown()
    except Exception as ex:
        print(ex)
        drive.shutdown()
    finally:  # this ensures a clean exit
        drive.shutdown()

