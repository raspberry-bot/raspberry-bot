import trio
import io

from picamera.array import PiRGBArray
from picamera import PiCamera

# import pygame.camera
# import pygame.image

from PIL import Image


class BaseSensor:
    def __init__(self):
        self.result = None
        self.name = self.__class__.__name__

    async def start(self):
        # clean up previous sensor data if any
        self.result = None

    async def read(self):
        self.result = 'a_value'

    async def stop(self):
        pass


class CameraSensor(BaseSensor):

    def __init__(self, index, width, height, quality):
        self.camera = PiCamera()
        self.raw_capture = PiRGBArray(self.camera)
        super(CameraSensor, self).__init__()

    def stop(self):
        self._cam.stop()

    def start(self):
        self._cam.start()

    def read(self):
        self.camera.capture(self.rawCapture, format="bgr")
        self.result = self.rawCapture.array

# class CameraSensor(BaseSensor):

#     def __init__(self, index, width, height, quality):
#         pygame.camera.init()
#         camera_name = pygame.camera.list_cameras()[index]
#         self._cam = pygame.camera.Camera(camera_name, (width, height))
#         self.quality = quality
#         super(CameraSensor, self).__init__()

#     def stop(self):
#         self._cam.stop()
#         self.is_started = False
#         self.stop_requested = False

#     def start(self):
#         self._cam.start()
#         self.is_started = True

#     def read(self):
#         img = self._cam.get_image()
#         imgstr = pygame.image.tostring(img, "RGB", False)
#         pimg = Image.frombytes("RGB", img.get_size(), imgstr)
#         with io.BytesIO() as bytesIO:
#             pimg.save(bytesIO, "JPEG", quality=self.quality, optimize=True)
#             self.result = bytesIO.getvalue()
#             return self.result
