import trio
import redis
import json
import base64
from sensors_service import SensorService

ss = SensorService()
for msg in ss.subscribe('CameraSensor').listen():
    if msg.get('type') == 'message':
        value = json.loads(msg.get('data'))
        raw_img = base64.b64decode(value.get('value'))
        print(value.get('ts'))
        print(raw_img)