import argparse
import json

import image as image
import tornado.httpserver, tornado.ioloop, tornado.options, tornado.web, os.path
from tornado.options import define, options

from keras.preprocessing import image
import numpy as np
import cv2
import uuid

define("port", default=8888, help="run on the given port", type=int)


class Application(tornado.web.Application):
    def __init__(self, model):
        self.model = model
        handlers = [
            (r"/predict-control", PredictControlHandler),
            (r"/vision-trainee", VisionTraineeHandler)
        ]
        tornado.web.Application.__init__(self, handlers)


class PredictControlHandler(tornado.web.RequestHandler):
    def post(self):
        image_data = self.request.files['image'][0]
        np_arr = np.fromstring(image_data['body'], np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_path = '/tmp/%s.png' % str(uuid.uuid4())
        cv2.imwrite(image_path, image_np)

        image_f = image.load_img(image_path, target_size=(64, 64))
        image_f = image.img_to_array(image_f)
        image_f = np.expand_dims(image_f, axis=0)

        left_or_right = self.application.model['left_or_right']
        straight_layer = self.application.model['straight']

        left_or_right_result = left_or_right.predict(image_f)
        straight_layer_result = straight_layer.predict(image_f)

        os.remove(image_path)

        self.write(json.dumps({
            'left_or_right': left_or_right_result,
            'straight': straight_layer_result
        }))


class VisionTraineeHandler(tornado.web.RequestHandler):
    def post(self):
        image_data = self.request.files['image'][0]

        data = self.request.body_arguments

        print(data)

        speed, steer = data['control']
        image_id = data['image_id'][0]

        np_arr = np.fromstring(image_data['body'], np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_path = 'data/input/%s.png' % str(image_id)
        image_abs_path = os.path.abspath(image_path)
        cv2.imwrite(image_abs_path, image_np)

        row = ','.join([speed, steer, image_id])

        self.update_dataset(row)
        self.write('ok')

    def update_dataset(self, row):
        with open('metadata.csv', 'a+') as metadata_f:
            metadata_f.write(row + '\n')



def main(args):
    PROJECT_PATH = '/Users/mehrdad/Documents/Dev/TheGreenBots/'
    # PROJECT_PATH = '/home/bot/catkin_ws/ml'

    if args.training:
        print('Collecting training data ...')
        http_server = tornado.httpserver.HTTPServer(Application({}))
        http_server.listen(options.port)
        tornado.ioloop.IOLoop.instance().start()

    elif args.predictive:
        from autonomous_engine import BinaryDecisionDriverLayer
        left_or_right_layer = BinaryDecisionDriverLayer(
            PROJECT_PATH,
            data_path='data/',
            model_name='left_or_right_layer',
            labels={0: 'left', 1: 'right'}
        )
        left_or_right_layer.load_model()
        print('Loaded %s model.' % left_or_right_layer.model_name)
        print(left_or_right_layer.model.summary())

        straight_layer = BinaryDecisionDriverLayer(
            PROJECT_PATH,
            data_path='data/',
            model_name='straight_layer',
            labels={1: 'straight', 0: 'left_or_right'}
        )
        straight_layer.load_model()
        print('Loaded %s model.' % straight_layer.model_name)
        print(straight_layer.model.summary())

        layers = {
            'straight': straight_layer,
            'left_or_right': left_or_right_layer
        }

        http_server = tornado.httpserver.HTTPServer(Application(layers))
        http_server.listen(options.port)
        tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Driver Server')
    parser.add_argument('--training', action="store_true", default=False)
    parser.add_argument('--predictive', action="store_true", default=False)
    args = parser.parse_args()
    main(args)

