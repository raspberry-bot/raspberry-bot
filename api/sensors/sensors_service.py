import trio
from api.sensors.sensor import BaseSensor, CameraSensor


class SensorService:
    def __init__(self):
        self.sensors = []

    def register(self, sensor):
        self.sensors.append(sensor)

    async def publish(self, channel, value):
        print(channel, value)

    async def run(self):
        while True:
            async with trio.open_nursery() as nursery:
                for sensor in self.sensors:
                    nursery.start_soon(sensor.start)
                    nursery.start_soon(sensor.read)
                    print('out of await')
                    await self.publish(sensor.name, sensor.result)


if __name__ == '__main__':
    ss = SensorService()
    ss.register(BaseSensor())
    ss.register(CameraSensor(width=640, height=480, quality=80))
    trio.run(ss.run)
