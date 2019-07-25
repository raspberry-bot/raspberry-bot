#!/usr/bin/env python3

import argparse

import tornado.ioloop
from tornado import web

from handlers.camera import CameraHandler
from handlers.driver import DriverHandler
from classes.driver import Driver
from classes.camera import Camera


class Application(web.Application):
    def __init__(self, camera, driver):
        self.camera = camera
        self.driver = driver
        handlers = [
            (r"/api/operate/camera", CameraHandler),
            (r"/api/operate/drive", DriverHandler),
        ]
        web.Application.__init__(self, handlers, debug=True)


def main(args):

    left_motor_pins = {
        'forward': 20,
        'reverse': 21,
        'control': 12
    }

    right_motor_pins = {
        'forward': 19,
        'reverse': 26,
        'control': 13
    }

    app = Application(
        camera=Camera(args.camera, args.width, args.height,
                      args.quality, args.stopdelay),
        driver=Driver(left_motor_pins, right_motor_pins)
    )
    app.listen(args.port)
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Start the PyImageStream server.')

    parser.add_argument('--port', default=8888, type=int,
                        help='Web server port (default: 8888)')
    parser.add_argument('--camera', default=0, type=int,
                        help='Camera index, first camera is 0 (default: 0)')
    parser.add_argument('--width', default=640, type=int,
                        help='Width (default: 640)')
    parser.add_argument('--height', default=480, type=int,
                        help='Height (default: 480)')
    parser.add_argument('--quality', default=100, type=int,
                        help='JPEG Quality 1 (worst) to 100 (best) (default: 100)')
    parser.add_argument('--stopdelay', default=20, type=int,
                        help='Delay in seconds before the camera will be stopped after all clients have disconnected (default: 20)')

    args = parser.parse_args()
    main(args)
