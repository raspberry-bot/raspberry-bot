from sensors_service import SensorService

import json
import base64

ss = SensorService()

drive_f = open('/opt/raspberry-bot/src/data/drive_commands.log', 'w+')
gyroscope_f = open('/opt/raspberry-bot/src/data/gyroscope.log', 'w+')

for msg in ss.subscribe(['CameraSensorData', 'Drive', 'GyroscopeSensorData']).listen():
    if msg['type'] == 'subscribe':
        if msg['data'] == 1:
            print('subscribed to: %s' % (msg['channel']))
    elif msg['type'] == 'message':
        value = json.loads(msg.get('data').decode("utf-8"))
        if value.get('channel') in ['CameraSensorData']:
            raw_img = base64.b64decode(value.get('value'))
            image_file_name = '/opt/raspberry-bot/src/data/images/' + str(value.get('ts')) + '.jpg'
            with open(image_file_name, 'wb+') as img_f:
                img_f.write(raw_img)
        elif 'Drive' in value.get('channel'):
            drive_f.write(str(value) + '\n')
        elif 'Gyroscope' in value.get('channel'):
            gyroscope_f.write(str(value) + '\n')
    else:
        print(msg)
