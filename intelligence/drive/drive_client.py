#!/usr/bin/env python


# Python libs
import argparse
import json
import sys
# Ros libraries
from uuid import uuid4

import RPi.GPIO as GPIO
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import requests

from distance_sensors import DistanceSensor

VERBOSE = True
TRAINING = True

class Base:
    def spin(self):
        rospy.spin()


class VisionDriverTrainee(Base):
    def __init__(self):
        self.distance_sensor = DistanceSensor()
        self.image = None
        self.data = None
        # self.webcam_subscriber = rospy.Subscriber("/webcam/image_raw/compressed", CompressedImage, self.image_callback)
        self.webcam_subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        self.control_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.control_callback)

    def image_callback(self, ros_data):
        self.image = ros_data.data

    def control_callback(self, ros_data):
        speed, steer = ros_data.linear.x, ros_data.angular.z
        if self.image:
            self.data = self.create_a_data_payload()
            self.data['control'] = speed, steer
            self.submit()
        else:
            return

    def submit(self):
        url = 'http://192.168.2.111:8888/vision-trainee'
        files = {
            'image': self.image
        }
        print(self.data)
        self.data['distance'] = self.distance_sensor.get_current_distance_to_obstacle()
        response = requests.post(url, data=self.data, files=files)
        if response.status_code == 200:
            self.image = None
            self.data = None

    def create_a_data_payload(self):
        return {
            'image_id': str(uuid4()),
            'control': None,
            'distance': None,
        }


class VisionDriver(Base):
    def __init__(self):
        self.webcam_subscriber = rospy.Subscriber("/webcam/image_raw/compressed", CompressedImage, self.img_callback)
        self.control_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move(self, speed, steer):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = steer
        self.control_publisher.publish(vel_msg)

    def img_callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''

        url = 'http://192.168.2.111:8888/predict-control'
        files = {'image': ros_data.data}
        response = requests.post(url, files=files)

        prediction = {
            'left_or_right': None,
            'straight': None
        }

        prediction = json.loads(response.content)

        print(prediction)

        if prediction['straight'] not in ['left_or_right'] and prediction['straight'] in ['straight']:
            self.move(0.26, 0)
        elif prediction['left_or_right']:
            if prediction['left_or_right'] in ['left']:
                self.move(0.26, 1.82)
            elif prediction['left_or_right'] in ['right']:
                self.move(0.26, -1.82)


def main(args):
    try:
        rospy.init_node('VisionDriverClient', anonymous=True)
        if args.training:
            print('VisionDriverTrainee initiated ...')
            vision_driver_trainee = VisionDriverTrainee()
            vision_driver_trainee.spin()
        elif args.real:
            print('VisionDriverTrainee initiated ...')
            vision_driver = VisionDriver()
            vision_driver.spin()

    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Driver Client')
    parser.add_argument('--training', action="store_true", default=False)
    parser.add_argument('--real', action="store_true", default=False)
    args = parser.parse_args()
    main(args)