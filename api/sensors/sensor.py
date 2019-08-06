import trio
import io
import pygame.camera
import pygame.image
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

    def __init__(self, camera_path="/dev/video0", width=640, height=480, quality=100):
        pygame.camera.init()
        self.camera_path = camera_path
        self.camera = pygame.camera.Camera(self.camera_path, (width, height))
        self.quality = quality
        super(CameraSensor, self).__init__()

    def stop(self):
        self.camera.stop()

    def start(self):
        self.camera.start()

    def read(self):
        img = self.camera.get_image()
        imgstr = pygame.image.tostring(img, "RGB", False)
        pimg = Image.frombytes("RGB", img.get_size(), imgstr)
        with io.BytesIO() as bytesIO:
            pimg.save(bytesIO, "JPEG", quality=self.quality, optimize=True)
            self.result = bytesIO.getvalue()
            return self.result
