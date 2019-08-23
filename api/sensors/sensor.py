import trio
import io
import pygame.camera
import pygame.image
from PIL import Image
import base64


class BaseSensor:
    def __init__(self):
        self.result = None
        self.started = False
        self.stopped = False
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
                return self.result


class CameraSensorBackend(CameraSensor):
    pass