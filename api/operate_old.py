import argparse
import json
import sys
import os.path
import subprocess
from datetime import datetime

import netifaces
from wifi import Cell, Scheme

import tornado.httpserver
import tornado.ioloop
from tornado import web
from tornado.options import define, options

import cv2
import base64
import RPi.GPIO as GPIO

import time

from server import BaseHandler

API_SERVER_ROOT = os.environ.get('API_SERVER_ROOT')
GREENBOTS_ROOT = os.environ.get('GREENBOTS_ROOT')

CONFIG_FILE = os.path.join(API_SERVER_ROOT, 'bot-config.json')
EVENTS_FILE = os.path.join(GREENBOTS_ROOT, 'logs/events.log')
VERSION_FILE = os.path.join(API_SERVER_ROOT, 'VERSION')


class Application(web.Application):
    def __init__(self, driver):
        self.camera = cv2.VideoCapture(0)
        self.driver = driver
        handlers = [
            (r"/api/operate/camera", CameraHandler),
            (r"/api/operate/control", ControlHandler),
        ]
        web.Application.__init__(self, handlers)

class Drive:

    """
    catkin_create_pkg drive std_msgs rospy roscpp sensor_msgs geometry_msgs cv_bridge
    """

    def __init__(self):
        self.mode = GPIO.getmode()

        self.IN1 = 19
        self.IN2 = 26
        self.IN3 = 20
        self.IN4 = 21

        self.control_a = 22
        self.control_b = 40

        self._setup_gpio()

        self.pwm_a = GPIO.PWM(self.control_a, 100)
        self.pwm_b = GPIO.PWM(self.control_b, 100)

        self.duty_cycle_increased = False

        self._setup_motors()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)

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
        GPIO.output(self.IN4, GPIO.HIGH)

    def left(self):
        print("Moving left...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)


class CameraHandler(BaseHandler):
    def get(self):
        retval, frame = self.application.camera.read()
        if retval != True:
            raise ValueError("Can't read frame")
        encoded_img = cv2.imencode('.jpeg', frame)[1]
        frame_b64 = base64.b64encode(encoded_img)
        self.write(frame_b64)


class ControlHandler(BaseHandler):
    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        if data == 'forward':
            self.application.driver.forward()
        elif data == 'reverse':
            self.application.driver.reverse()
        elif data == 'left':
            self.application.driver.left()
        elif data == 'right':
            self.application.driver.right()
        elif data == 'stop':
            self.application.driver.stop()
        self.write(json.dumps(data))


def main(args):
    define("port", default=args.port, help="Run on the given port", type=int)
    http_api = tornado.httpserver.HTTPServer(Application(
        driver = Drive()
    ))
    http_api.listen(options.port)
    tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Web Interface api')

    parser.add_argument('--port', type=int, help="port to run on. Must be supplied.")
    args = parser.parse_args()
    main(args)
