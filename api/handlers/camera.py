import tornado.websocket


class CameraHandler(tornado.websocket.WebSocketHandler):
    clients = set()
    def initialize(self):
        self.camera_channel = self.application.sensors_service.subscribe('CameraSensor')

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        CameraHandler.clients.add(self)
        print("WebSocket opened from: " + self.request.remote_ip)

    def on_message(self, message):
        msg = self.camera_channel.get_message()
        if msg.get('type') in ['message']:
            value = json.loads(msg.get('data'))
            raw_img = base64.b64decode(value.get('value'))
            self.write_message(raw_img, binary=True)

    def load_a_new_frame(self):
        msg = self.camera_channel.get_message()
        if msg.get('type') in ['message']:
            value = json.loads(msg.get('data'))
            raw_img = base64.b64decode(value.get('value'))
            for client in CameraHandler.clients:
                client.write_message(raw_img, binary=True)

    def on_close(self):
        CameraHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)

        stopdelay = self.application.camera.stopdelay
        camera_stop_function = self.application.camera.stop

        if len(CameraHandler.clients) == 0:
            if self.application.camera.request_stop():
                tornado.ioloop.IOLoop.current().call_later(stopdelay, camera_stop_function)
