import tornado.websocket
import json


class DriverHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        DriverHandler.clients.add(self)
        print("WebSocket opened from: " + self.request.remote_ip)

    def on_message(self, message):
        print('Receiveed msg from Driver Websocket: %s' % message)
        if message == 'forward':
            state = self.application.driver.forward()
        elif message == 'reverse':
            state = self.application.driver.reverse()
        elif message == 'left':
            state = self.application.driver.left()
        elif message == 'right':
            state = self.application.driver.right()
        elif message == 'stop':
            state = self.application.driver.stop()

        self.write_message(json.dumps(state))

    def on_close(self):
        DriverHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(DriverHandler.clients) == 0:
            self.application.driver.stop()
