import trio
from api.sensors.sensor import BaseSensor, CameraSensor
import redis
import time
import json


class SensorService:
    def __init__(self):
        self.sensors = []
        self.redis = redis.Redis(host='localhost', port=6379, db=0)

    def register(self, sensor):
        self.sensors.append(sensor)

    def subscribe(self, channel):
        pubsub = self.redis.pubsub()
        pubsub.subscribe([channel])
        while True:
            msg = pubsub.get_message()
            if msg:
                print(msg)
                yield msg

    async def publish(self, channel, value, timestamp_ms):
        message = {
            'timestamp': timestamp_ms,
            'value': value
        }
        # print((channel, message.get('timestamp'), message.get('value', [])))
        print(('sending data to ', channel, timestamp_ms))
        self.redis.publish(channel, json.dumps(message))

    async def run(self):
        while True:
            async with trio.open_nursery() as nursery:
                timestamp_ms = int(time.time() * 1000)
                for sensor in self.sensors:
                    nursery.start_soon(sensor.start)
                    nursery.start_soon(sensor.read)
                    await self.publish(sensor.name, sensor.result, timestamp_ms)


if __name__ == '__main__':
    ss = SensorService()
    ss.register(BaseSensor())
    ss.register(CameraSensor(width=640, height=480, quality=80))
    trio.run(ss.run)
