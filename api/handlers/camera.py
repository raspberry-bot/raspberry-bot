import tornado.websocket


class CameraHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        CameraHandler.clients.add(self)
        print("WebSocket opened from: " + self.request.remote_ip)
        self.application.camera.request_start()

    def on_message(self, message):
        jpeg_bytes = self.application.camera.get_jpeg_image_bytes()
        self.write_message(jpeg_bytes, binary=True)

    def on_close(self):
        CameraHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)

        stopdelay = self.application.camera.stopdelay
        camera_stop_function = self.application.camera.stop

        if len(CameraHandler.clients) == 0:
            if self.application.camera.request_stop():
                tornado.ioloop.IOLoop.current().call_later(stopdelay, camera_stop_function)
