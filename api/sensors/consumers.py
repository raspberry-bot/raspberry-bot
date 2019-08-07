import trio
import redis
import json
import base64
from sensors_service import SensorService

ss = SensorService()
for msg in ss.subscribe('CameraSensor'):
    msg = json.loads(msg)
    raw_img = base64.decode(msg.get('value'))
    print(msg.get('ts'))
    print(raw_img)