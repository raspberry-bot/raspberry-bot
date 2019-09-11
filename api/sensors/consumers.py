import json
import base64
import cv2
import urllib.request
import numpy as np
import time
import argparse

from sensors_service import SensorService

ss = SensorService()

drive_f = open('/opt/raspberry-bot/src/data/drive.log', 'w+')
gyroscope_f = open('/opt/raspberry-bot/src/data/gyroscope.log', 'w+')


def sensors():
    for msg in ss.subscribe(['CameraSensorData', 'DriveData', 'GyroscopeSensorData']).listen():
        if msg['type'] == 'subscribe':
            if msg['data'] == 1:
                print('subscribed to: %s' % (msg['channel']))
        elif msg['type'] == 'message':
            value = json.loads(msg.get('data').decode("utf-8"))
            if 'CameraSensorData' in value.get('channel'):
                raw_img = base64.b64decode(value.get('value'))
                img_file_path = '/opt/raspberry-bot/src/data/images/' + str(value.get('ts')) + '.jpg'
                store_image(img_file_path)
            elif 'DriveData' in value.get('channel'):
                drive_f.write(str(value) + '\n')
            elif 'GyroscopeSensorData' in value.get('channel'):
                gyroscope_f.write(str(value) + '\n')
        else:
            print(msg)


def store_image():
    with urllib.request.urlopen('http://raspberrybot.local:5000/?action=stream') as stream:
        img_file_path = '/opt/raspberry-bot/src/data/images/' + str(int(time.time() * 1000)) + '.jpg'
        img_bytes = b""
        while True:
            img_bytes += stream.read(1024)
            a = img_bytes.find(b"\xff\xd8")
            b = img_bytes.find(b"\xff\xd9")
            if a != -1 and b != -1:
                jpg = img_bytes[a:b+2]
                img_bytes = img_bytes[b+2:]
                image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
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