import trio
import io
import pygame.camera
import pygame.image
from PIL import Image
import base64
import smbus
import math


class BaseSensor:
    def __init__(self):
        self.result = None
        self.started = False
        self.stopped = False
        self.redis = None
        self.name = self.__class__.__name__

    async def start(self):
        if not self.started:
            self.result = None
            self.started = True

    async def read(self):
        if self.started:
            self.result = 'a_value'

    async def stop(self):
        if not self.stopped:
            self.stopped = True


class CameraSensor(BaseSensor):

    def __init__(self, camera_path="/dev/video0", width=640, height=480, quality=100):
        pygame.camera.init()
        self.camera_path = camera_path
        self.camera = pygame.camera.Camera(self.camera_path, (width, height))
        self.quality = quality
        super(CameraSensor, self).__init__()

    async def stop(self):
        if not self.stopped:
            self.camera.stop()
            self.stopped = True

    async def start(self):
        if not self.started:
            self.camera.start()
            self.started = True

    async def read(self):
        if self.started:
            img = self.camera.get_image()
            imgstr = pygame.image.tostring(img, "RGB", False)
            pimg = Image.frombytes("RGB", img.get_size(), imgstr)
            with io.BytesIO() as bytesIO:
                pimg.save(bytesIO, "JPEG", quality=self.quality, optimize=True)
                # self.result = base64.b64encode(bytesIO.getvalue().encode())
                self.result = base64.b64encode(bytesIO.getvalue()).decode()
                self.publish(self.name, self.result)
                self.publish(self.name + 'Data', self.result)


class GyroscopeSensor(BaseSensor):

    def __init__(self):
        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
        self.bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
        self.address = 0x68       # via i2cdetect
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
        super(GyroscopeSensor, self).__init__()

    async def stop(self):
        if not self.stopped:
            self.stopped = True

    async def start(self):
        if not self.started:
            self.started = True

    async def read(self):
        if self.started:
            payload = GyroscopeSensor.get_empty_payload_dict()
            payload['gyroscope']['out']['x'] = self._read_word_2c(0x43)
            payload['gyroscope']['out']['y'] = self._read_word_2c(0x45)
            payload['gyroscope']['out']['z'] = self._read_word_2c(0x47)
            payload['gyroscope']['scale']['x'] = payload['gyroscope']['out']['x'] / 131
            payload['gyroscope']['scale']['y'] = payload['gyroscope']['out']['y'] / 131
            payload['gyroscope']['scale']['z'] = payload['gyroscope']['out']['z'] / 131

            payload['accelerometer']['out']['x'] = self._read_word_2c(0x3b)
            payload['accelerometer']['out']['y'] = self._read_word_2c(0x3d)
            payload['accelerometer']['out']['z'] = self._read_word_2c(0x3f)
            payload['accelerometer']['scale']['x'] = payload['accelerometer']['out']['x'] / 16384.0
            payload['accelerometer']['scale']['y'] = payload['accelerometer']['out']['y'] / 16384.0
            payload['accelerometer']['scale']['z'] = payload['accelerometer']['out']['z'] / 16384.0
            
            payload['rotation']['x'] = self._get_x_rotation(
                payload['accelerometer']['scale']['x'], 
                payload['accelerometer']['scale']['y'], 
                payload['accelerometer']['scale']['z']
            )
            payload['rotation']['y'] = self._get_y_rotation(
                payload['accelerometer']['scale']['x'], 
                payload['accelerometer']['scale']['y'], 
                payload['accelerometer']['scale']['z']
            )

            self.publish(self.name, payload)
            self.publish(self.name + 'Data', payload)
    
    def _read_byte(self, reg):
        return self.bus.read_byte_data(address, reg)
 
    def _read_word(self, reg):
        h = self.bus.read_byte_data(address, reg)
        l = self.bus.read_byte_data(address, reg+1)
        value = (h << 8) + l
        return value
    
    def _read_word_2c(self, reg):
        val = self._read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
    
    def _dist(self, a, b):
        return math.sqrt((a*a)+(b*b))
    
    def _get_y_rotation(self, x,y,z):
        radians = math.atan2(x, self._dist(y,z))
        return -math.degrees(radians)
    
    def _get_x_rotation(self, x,y,z):
        radians = math.atan2(y, self._dist(x,z))
        return math.degrees(radians)
    
    @staticmethod
    def get_empty_payload_dict():
        return {
            'rotation': {
                'x': 0,
                'y': 0
            },
            'accelerometer': {
                'out': {
                    'x': 0,
                    'y': 0,
                    'z': 0
                },
                'scale': {
                    'x': 0,
                    'y': 0,
                    'z': 0
                }
            },
            'gyroscope': {
                'out': {
                    'x': 0,
                    'y': 0,
                    'z': 0
                },
                'scale': {
                    'x': 0,
                    'y': 0,
                    'z': 0
                }
            }
        }