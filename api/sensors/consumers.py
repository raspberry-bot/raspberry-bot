from sensors_service import SensorService

import json
import base64

ss = SensorService()
for msg in ss.subscribe('CameraSensor').listen():
    if msg['type'] == 'subscribe':
        if msg['data'] == 1:
            log.info('subscribed to: %s' % (msg['channel']))
    elif msg['type'] == 'message':
        value = json.loads(msg.get('data'))
        raw_img = base64.b64decode(value.get('value'))
        print(value.get('ts'))
        print(raw_img)
    else:
        print m
