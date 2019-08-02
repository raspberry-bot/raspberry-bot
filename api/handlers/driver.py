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
        # print('Receiveed msg from Driver Websocket: %s' % message)
        x, y = 0, 0
        min_speed, max_speed = -100, 100
        left_speed, right_speed = 0, 0
        try:
            data = tornado.escape.json_decode(message)
            x = int(data['x'])
            y = int(data['y'])
            min_speed = int(data['min_speed'])
            max_speed = int(data['max_speed'])
            if x and y:
                left_speed, right_speed = self.application.driver.command_to_diff(x, y, min_speed, max_speed)
        except Exception as ex:
            print(ex)
        finally:
            self.application.driver.left_motor.move(left_speed)
            self.application.driver.right_motor.move(right_speed)
            self.write_message(json.dumps({'left_speed': left_speed, 'right_speed': right_speed}))

    # def on_message(self, message):
    #     print('Receiveed msg from Driver Websocket: %s' % message)
    #     if message == 'forward':
    #         state = self.application.driver.forward()
    #     elif message == 'reverse':
    #         state = self.application.driver.reverse()
    #     elif message == 'left':
    #         state = self.application.driver.left()
    #     elif message == 'right':
    #         state = self.application.driver.right()
    #     elif message == 'stop':
    #         state = self.application.driver.stop()

    #     self.write_message(json.dumps(state))

    def on_close(self):
        DriverHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)
        if len(DriverHandler.clients) == 0:
            self.application.driver.left_motor.move(0)
            self.application.driver.right_motor.move(0)
