#!/usr/bin/env python3

import argparse

import tornado.ioloop
from tornado import web

from api.handlers.camera import CameraHandler
from api.handlers.driver import DriverHandler

from api.classes.camera import Camera
from api.classes.driver import Driver

from api.sensors.sensors_service import SensorService


class Application(web.Application):
    def __init__(self, sensors_service, driver):
        self.sensors_service = sensors_service
        self.driver = driver
        handlers = [
            (r"/api/operate/camera", CameraHandler),
            (r"/api/operate/drive", DriverHandler),
        ]
        web.Application.__init__(self, handlers, debug=True)


def main(args):
    app = Application(
        sensors_service=SensorService(),
        driver=Driver()
    )
    app.listen(args.port)
    CHECK_INTERVAL = 10  # ms
    ioloop.PeriodicCallback(CameraHandler.load_a_new_frame, CHECK_INTERVAL).start()
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Start the Operate server.')

    parser.add_argument('--port', default=8888, type=int, help='Web server port (default: 8888)')
    args = parser.parse_args()
    main(args)
