import trio
from api.sensors.sensor import BaseSensor, CameraSensor
import redis
import time
import json


class SensorService:
    def __init__(self, host='localhost'):
        self.sensors = []
        self.redis = redis.Redis(host=host, port=6379, db=0)

    def register(self, sensor):
        self.sensors.append(sensor)

    def subscribe(self, channel):
        pubsub = self.redis.pubsub()
        pubsub.subscribe(channel)
        return pubsub

    def publish(self, channel, value):
        message = {
            'ts': str(int(time.time() * 1000)),
            'value': value,
            'channel': channel
        }
        self.redis.publish(channel, json.dumps(message))

    async def run(self):
        while True:
            async with trio.open_nursery() as nursery:
                for sensor in self.sensors:
                    sensor.publish = self.publish
                    nursery.start_soon(sensor.start)
                    nursery.start_soon(sensor.read)


if __name__ == '__main__':
    sensors = [
        BaseSensor(),
        # CameraSensor(width=640, height=480, quality=80)
    ]

    ss = SensorService()
    for sensor in sensors:
        ss.register(sensor())

    if len(sensors) > 1:
        trio.run(ss.run)
