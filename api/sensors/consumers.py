from sensors_service import SensorService

import json
import base64

ss = SensorService()
for msg in ss.subscribe(['CameraSensorBackend', 'Drive']).listen():
    if msg['type'] == 'subscribe':
        if msg['data'] == 1:
            print('subscribed to: %s' % (msg['channel']))
    elif msg['type'] == 'message':
        if msg.get('data').get('channel') in ['CameraSensorBackend']:
            value = json.loads(msg.get('data'))
            raw_img = base64.b64decode(value.get('value'))
        
    else:
        print(msg)
