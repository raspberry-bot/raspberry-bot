import argparse
import json
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
# import rospy
# from geometry_msgs.msg import Twist

CONFIG_FILE = '/opt/thegreenbot/config/config.json'
EVENTS_FILE = '/opt/thegreenbot/logs/events.log'
VERSION_FILE = '/opt/thegreenbot/VERSION'


class Application(web.Application):
    def __init__(self, model):
        self.camera = cv2.VideoCapture(0)
        # self.control_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.model = model
        handlers = [
            (r"/api/operate/camera", CameraHandler),
            (r"/api/operate/control", ControlHandler),
        ]
        web.Application.__init__(self, handlers)


class BaseHandler(web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')


class CameraHandler(BaseHandler):
    def get(self):
        retval, frame = self.application.camera.read()
        if retval != True:
            raise ValueError("Can't read frame")
        encoded_img = cv2.imencode('.png',frame)[1]
        frame_b64 = base64.b64encode(encoded_img)
        self.write(frame_b64)


class ControlHandler(BaseHandler):
    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        # vel_msg = Twist()
        # vel_msg.linear.x = abs(data)
        # vel_msg.linear.x = -abs(data)
        # vel_msg.angular.z = data
        # self.application.control_publisher.publish(vel_msg)
        self.write(json.dumps(data))


def main(args):
    define("port", default=args.port, help="Run on the given port", type=int)
    http_api = tornado.httpserver.HTTPServer(Application({}))
    http_api.listen(options.port)
    tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Web Interface api')

    parser.add_argument('--port', type=int, help="port to run on. Must be supplied.")
    args = parser.parse_args()
    main(args)
