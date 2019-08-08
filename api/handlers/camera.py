import tornado.websocket
import tornado.ioloop


class CameraHandler(tornado.websocket.WebSocketHandler):
    clients = set()
    def initialize(self):
        self.camera_channel = self.application.sensors_service.subscribe('CameraSensor')
        self.callback = tornado.ioloop.PeriodicCallback(self.send_a_new_frame, 10)

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        CameraHandler.clients.add(self)
        self.callback.start()
        print("WebSocket opened from: " + self.request.remote_ip)

    def on_message(self, message):
        self.write_message(self._get_a_new_frame(), binary=True)

    def send_a_new_frame(self):
        raw_img = self._get_a_new_frame()
        for client in CameraHandler.clients:
            client.write_message(raw_img, binary=True)

    def _get_a_new_frame(self):
        msg = self.camera_channel.get_message()
        if msg and msg.get('type') in ['message']:
            value = json.loads(msg.get('data'))
            raw_img = base64.b64decode(value.get('value'))
            return raw_img

    def on_close(self):
        CameraHandler.clients.remove(self)
        self.callback.stop()
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(CameraHandler.clients) == 0:
            pass
