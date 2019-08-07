import trio
import redis
from sensors_service import SensorService

def logit(message):
    print(message)

ss = SensorService()
trio.run(ss.subscribe, *('Camera*', logit))