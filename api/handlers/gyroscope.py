import tornado.websocket
import tornado.ioloop
import json


class GyroscopeSensorHandler(tornado.websocket.WebSocketHandler):
    clients = set()
    def initialize(self):
        self.gyroscope_channel = self.application.sensors_service.subscribe('GyroscopeSensor')
        self.callback = tornado.ioloop.PeriodicCallback(self.send_a_new_payload, 1)

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        GyroscopeSensorHandler.clients.add(self)
        self.callback.start()
        print("WebSocket opened from: " + self.request.remote_ip)

    def on_message(self, message):
        pass

    def send_a_new_payload(self):
        payload = self._get_a_new_payload()
        for client in GyroscopeSensorHandler.clients:
            client.write_message(json.dumps(payload))

    def _get_a_new_payload(self):
        msg = self.gyroscope_channel.get_message()
        if msg and msg.get('type') in ['message']:
            value = json.loads(msg.get('data'))
            if value.get('value') is not None:
                return value.get('value')

    def on_close(self):
        GyroscopeSensorHandler.clients.remove(self)
        self.callback.stop()
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(GyroscopeSensorHandler.clients) == 0:
            pass
