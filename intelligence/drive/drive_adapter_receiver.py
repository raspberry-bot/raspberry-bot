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

        self.pinMotorAForward = 16
        self.pinMotorABackward = 36
        self.pinMotorBForward = 18
        self.pinMotorBBackward = 38

        self.pinMotorAControl = 22
        self.pinMotorBControl = 40

        # self.control_a = 22
        # self.control_b = 40

        self.frequency = 10000
        self.stop_value = 0.0
        self.duty_cycle = 50.0

        self.duty_cycle_increased = False

        self._setup_gpio()
        self._setup_motors()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.pinMotorAForward, GPIO.OUT)
        GPIO.setup(self.pinMotorABackward, GPIO.OUT)
        GPIO.setup(self.pinMotorBForward, GPIO.OUT)
        GPIO.setup(self.pinMotorBBackward, GPIO.OUT)

        GPIO.setup(self.pinMotorAControl, GPIO.OUT)
        GPIO.setup(self.pinMotorBControl, GPIO.OUT)

        self.pwmMotorAControl = GPIO.PWM(self.pinMotorAControl, self.frequency)
        self.pwmMotorBControl = GPIO.PWM(self.pinMotorBControl, self.frequency)

        # self.pwm_b = GPIO.PWM(self.control_b, self.frequency)

        # GPIO.setup(self.control_a, GPIO.OUT)
        # GPIO.setup(self.control_b, GPIO.OUT)

    def _setup_motors(self):
        # if self.duty_cycle_increased is False:
        #     for pct in range(100):
        #         self.pwmMotorAForward.start(pct)
        #         time.sleep(1.0/1000)
        #     print('Setting ChangeDutyCycle to 25...')
        #     self.pwmMotorAForward.start(25)
        #     self.pwm_b.start(25)

        #     time.sleep(1)
        #     print('Increasing ChangeDutyCycle to 100...')
        #     self.pwmMotorAForward.ChangeDutyCycle(100)
        #     self.pwm_b.ChangeDutyCycle(100)
        #     self.duty_cycle_increased = True
        self.pwmMotorAControl.start(self.stop_value)
        self.pwmMotorBControl.start(self.stop_value)

    def stop(self):
        GPIO.output(self.pinMotorAForward, GPIO.LOW)
        GPIO.output(self.pinMotorABackward, GPIO.LOW)
        GPIO.output(self.pinMotorBForward, GPIO.LOW)
        GPIO.output(self.pinMotorBBackward, GPIO.LOW)
        
        self.pwmMotorAControl.start(self.stop_value)
        self.pwmMotorBControl.start(self.stop_value)

    def forward(self):
        print("Moving forward...")
        GPIO.output(self.pinMotorAForward, GPIO.HIGH)
        GPIO.output(self.pinMotorABackward, GPIO.HIGH)
        GPIO.output(self.pinMotorBForward, GPIO.HIGH)
        GPIO.output(self.pinMotorBBackward, GPIO.HIGH)
        
        self.pwmMotorAControl.start(self.duty_cycle)
        self.pwmMotorBControl.start(self.duty_cycle)

    def reverse(self):
        print("Moving reverse...")
        self.pwmMotorAForward.ChangeDutyCycle(self.stop)
        self.pwmMotorABackward.ChangeDutyCycle(self.duty_cycle)
        self.pwmMotorBForward.ChangeDutyCycle(self.stop)
        self.pwmMotorBBackward.ChangeDutyCycle(self.duty_cycle)

    def right(self):
        print("Moving right...")
        print(self.pwmMotorAForward)
        self.pwmMotorAForward.ChangeDutyCycle(self.duty_cycle)
        self.pwmMotorABackward.ChangeDutyCycle(self.stop)
        self.pwmMotorBForward.ChangeDutyCycle(self.stop)
        self.pwmMotorBBackward.ChangeDutyCycle(self.duty_cycle)

    def left(self):
        print("Moving left...")
        self.pwmMotorAForward.ChangeDutyCycle(self.stop)
        self.pwmMotorABackward.ChangeDutyCycle(self.duty_cycle)
        self.pwmMotorBForward.ChangeDutyCycle(self.duty_cycle)
        self.pwmMotorBBackward.ChangeDutyCycle(self.stop)

    def ros_adapter(self, data):
        speed = data.linear.x
        steer = data.angular.z
        print(speed, steer)
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
        GPIO.clean()

