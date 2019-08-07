import trio
import redis
import json
import base64
from sensors_service import SensorService

ss = SensorService()
for msg in ss.subscribe('CameraSensor'):
    import pdb;pdb.set_trace()
    value = json.loads(msg.get('data'))
    raw_img = base64.decode(value)
    print(value.get('ts'))
    print(raw_img)