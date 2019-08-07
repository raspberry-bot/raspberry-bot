import trio
import redis
from sensors_service import SensorService

ss = SensorService()
for item in ss.stream('CameraSensor'):
    print(('consumer -->', item))