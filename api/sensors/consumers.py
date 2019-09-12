import json
import base64
import cv2
import urllib.request
import numpy as np
import time
import argparse
import os

from sensors_service import SensorService

ss = SensorService()

DATA_PATH = '/opt/raspberry-bot/src/data/'

drive_f = open(os.path.join(DATA_PATH, 'drive.log'), 'w+')
gyroscope_f = open(os.path.join(DATA_PATH, 'gyroscope.log'), 'w+')


def sensors():
    for msg in ss.subscribe(['DriveData', 'GyroscopeSensorData']).listen():
        if msg['type'] == 'subscribe':
            if msg['data'] == 1:
                print('subscribed to: %s' % (msg['channel']))
        elif msg['type'] == 'message':
            value = json.loads(msg.get('data').decode("utf-8"))
            if 'DriveData' in value.get('channel'):
                drive_f.write(str(value) + '\n')
                drive_f.flush()
            elif 'GyroscopeSensorData' in value.get('channel'):
                gyroscope_f.write(str(value) + '\n')
                gyroscope_f.flush()
        else:
            print(msg)


def store_image():
    with urllib.request.urlopen('http://raspberrybot.local:5000/?action=stream') as stream:
        img_file_path = open(os.path.join(DATA_PATH,'images/' + str(int(time.time() * 1000)) + '.jpg'), 'w+')
        img_bytes = b""
        while True:
            img_bytes += stream.read(1024)
            a = img_bytes.find(b"\xff\xd8")
            b = img_bytes.find(b"\xff\xd9")
            if a != -1 and b != -1:
                jpg = img_bytes[a:b+2]
                img_bytes = img_bytes[b+2:]
                image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                cv2.imwrite(img_file_path, image)
                return True


def main(args):
    if args.camera:
        while True:
            store_image()
    elif args.sensors:
        while True:
            sensors()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Consumer')
    parser.add_argument('--camera', action='store_true')
    parser.add_argument('--sensors', action='store_true')
    args = parser.parse_args()
    main(args)