import trio
import redis
from sensors_service import SensorService

ss = SensorService()
while True:
    ss.get('Camera*')