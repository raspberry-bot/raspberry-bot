#!/usr/bin/env python3

import argparse

import tornado.ioloop
from tornado import web

from api.handlers.gyroscope import GyroscopeSensorHandler

from api.sensors.sensors_service import SensorService


class Application(web.Application):
    def __init__(self, sensors_service):
        self.sensors_service = sensors_service
        handlers = [
            (r"/api/sensors/gyroscope", GyroscopeSensorHandler),
        ]
        web.Application.__init__(self, handlers, debug=True)


def main(args):
    app = Application(
        sensors_service=SensorService()
    )
    app.listen(args.port)
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Start the Operate server.')

    parser.add_argument('--port', default=8889, type=int, help='Web server port (default: 8888)')
    args = parser.parse_args()
    main(args)
