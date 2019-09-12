import tornado.websocket
import tornado.ioloop
import json


class DistanceMeterSensorHandler(tornado.websocket.WebSocketHandler):
    clients = set()
    def initialize(self):
        self.distance_channel = self.application.sensors_service.subscribe('DistanceMeterSensor')
        self.callback = tornado.ioloop.PeriodicCallback(self.send_a_new_payload, 1)

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        DistanceMeterSensorHandler.clients.add(self)
        self.callback.start()
        print("WebSocket opened from: " + self.request.remote_ip)

    def on_message(self, message):
        pass

    def send_a_new_payload(self):
        payload = self._get_a_new_payload()
        for client in DistanceMeterSensorHandler.clients:
            client.write_message(json.dumps(payload))

    def _get_a_new_payload(self):
        msg = self.distance_channel.get_message()
        if msg and msg.get('type') in ['message']:
            value = json.loads(msg.get('data'))
            if value.get('value') is not None:
                return value.get('value')

    def on_close(self):
        DistanceMeterSensorHandler.clients.remove(self)
        self.callback.stop()
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(DistanceMeterSensorHandler.clients) == 0:
            pass
