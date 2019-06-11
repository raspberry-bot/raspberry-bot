import sys, time
import random
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
from geometry_msgs.msg import Twist
from time import sleep


def move(speed, steer):
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = speed
    vel_msg.angular.z = steer
    publisher.publish(vel_msg)


def main():
	while True:
		speed, steer = random.uniform(0, 0.27), random.uniform(-2.8,2.8)
		move(speed, steer)
		print((speed, steer))
		sleep(0.1)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down ROS Image feature detector module")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()