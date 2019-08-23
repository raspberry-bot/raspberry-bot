from sensors_service import SensorService

import json
import base64

ss = SensorService()
drive_f = open('../../data/drive_commands.log', 'a+')
for msg in ss.subscribe(['CameraSensorData', 'Drive']).listen():
    if msg['type'] == 'subscribe':
        if msg['data'] == 1:
            print('subscribed to: %s' % (msg['channel']))
    elif msg['type'] == 'message':
        value = json.loads(msg.get('data'))
        if msg.get('data').get('channel') in ['CameraSensorData']:
            raw_img = base64.b64decode(value.get('value'))
            with open('../../data/images/' + str(value.get('ts')) + '.jpg', 'w+') as img_f:
                img_f.write(raw_img)
        elif msg.get('data').get('channel') in ['Drive']:
            drive_f.write(str(value) + '\n')
    else:
        print(msg)
