#!/usr/bin/env python3

import argparse
import os
import io
import json
import sys
import time

import tornado.ioloop
import tornado.websocket
from tornado import web

import pygame.camera
import pygame.image

from PIL import Image
import RPi.GPIO as GPIO

from server import BaseHandler



class Camera:

    def __init__(self, index, width, height, quality, stopdelay):
        print("Initializing camera...")
        pygame.camera.init()
        camera_name = pygame.camera.list_cameras()[index]
        self._cam = pygame.camera.Camera(camera_name, (width, height))
        print("Camera initialized")
        self.is_started = False
        self.stop_requested = False
        self.quality = quality
        self.stopdelay = stopdelay

    def request_start(self):
        if self.stop_requested:
            print("Camera continues to be in use")
            self.stop_requested = False
        if not self.is_started:
            self._start()

    def request_stop(self):
        if self.is_started and not self.stop_requested:
            self.stop_requested = True
            print("Stopping camera in " + str(self.stopdelay) + " seconds...")
            tornado.ioloop.IOLoop.current().call_later(self.stopdelay, self._stop)

    def _start(self):
        print("Starting camera...")
        self._cam.start()
        print("Camera started")
        self.is_started = True

    def _stop(self):
        if self.stop_requested:
            print("Stopping camera now...")
            self._cam.stop()
            print("Camera stopped")
            self.is_started = False
            self.stop_requested = False

    def get_jpeg_image_bytes(self):
        img = self._cam.get_image()
        imgstr = pygame.image.tostring(img, "RGB", False)
        pimg = Image.frombytes("RGB", img.get_size(), imgstr)
        with io.BytesIO() as bytesIO:
            pimg.save(bytesIO, "JPEG", quality=self.quality, optimize=True)
            return bytesIO.getvalue()


class CameraHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        CameraHandler.clients.add(self)
        print("WebSocket opened from: " + self.request.remote_ip)
        self.application.camera.request_start()

    def on_message(self, message):
        jpeg_bytes = self.application.camera.get_jpeg_image_bytes()
        self.write_message(jpeg_bytes, binary=True)

    def on_close(self):
        CameraHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(CameraHandler.clients) == 0:
            self.application.camera.request_stop()


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

    def left(self):
        print("Moving left...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def right(self):
        print("Moving right...")
        self._setup_motors()
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)


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



class Application(web.Application):
    def __init__(self, camera, driver):
        self.camera = camera
        self.driver = driver
        handlers = [
            (r"/api/operate/camera", CameraHandler),
            (r"/api/operate/control", ControlHandler),
        ]
        web.Application.__init__(self, handlers)

def main(args):
    app = Application(
        camera = Camera(args.camera, args.width, args.height, args.quality, args.stopdelay),
        driver = Drive()
    )
    app.listen(args.port)
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Start the PyImageStream server.')

    parser.add_argument('--port', default=8888, type=int, help='Web server port (default: 8888)')
    parser.add_argument('--camera', default=0, type=int, help='Camera index, first camera is 0 (default: 0)')
    parser.add_argument('--width', default=640, type=int, help='Width (default: 640)')
    parser.add_argument('--height', default=480, type=int, help='Height (default: 480)')
    parser.add_argument('--quality', default=70, type=int, help='JPEG Quality 1 (worst) to 100 (best) (default: 70)')
    parser.add_argument('--stopdelay', default=7, type=int, help='Delay in seconds before the camera will be stopped after all clients have disconnected (default: 7)')

    args = parser.parse_args()
    main(args)
