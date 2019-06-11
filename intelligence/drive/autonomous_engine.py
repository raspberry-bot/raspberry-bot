import os
import numpy as np


from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten, Dropout
from keras.layers import Conv2D, MaxPooling2D
from keras import optimizers, Input, Model
from keras_preprocessing import image
from keras_preprocessing.image import ImageDataGenerator
from tensorflow.python.keras.engine.saving import model_from_json

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


class BaseDriverLayer:

    def __init__(self, project_path, data_path, model_name, labels):
        self.model_name = model_name
        self.project_path = project_path
        self.data_path = os.path.join(project_path, data_path)
        self.labels = labels
        self.model = self.build_model()
        print('Instantiating a model...')
        print({
            'project_path': self.project_path,
            'data_path': self.data_path,
            'labels': self.labels,
            'model_name': self.model_name,
        })

    def build_model(self):
        pass

    def train(self):
        pass

    def predict(self, **kwargs):
        pass

    def load_image_from_disk(self, path):
        abs_path = os.path.join(self.project_path, path)
        print('loading %s ...' % abs_path)
        image_f = image.load_img(abs_path, target_size=(64, 64))
        image_array = image.img_to_array(image_f)
        image_array = np.expand_dims(image_array, axis=0)
        return image_array

    def save_model(self):
        print('Saving model...')
        # serialize model to JSON
        model_json = self.model.to_json()
        with open(os.path.join(self.data_path, 'models', '%s.json' % self.model_name), "w") as json_file:
            json_file.write(model_json)
        # serialize weights to HDF5
        self.model.save_weights(os.path.join(self.data_path, 'models', '%s.h5' % self.model_name))
        print("Saved model to disk")

    def load_model(self):
        # load json and create model
        json_file = open(os.path.join(self.data_path, 'models', '%s.json' % self.model_name), 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        loaded_model = model_from_json(loaded_model_json)
        # load weights into new model
        loaded_model.load_weights(os.path.join(self.data_path, 'models', '%s.h5' % self.model_name))
        print("Loaded model from disk")
        self.model = loaded_model
        return loaded_model


class BinaryDecisionDriverLayer(BaseDriverLayer):

    def build_model(self):
        print('Building model: %s' % self.model_name)
        self.model = Sequential()
        self.model.add(Conv2D(32, (3, 3), input_shape=(64, 64, 3), activation='relu'))

        # Step 2 - Pooling
        self.model.add(MaxPooling2D(pool_size=(2, 2)))

        # Adding a second convolutional layer
        self.model.add(Conv2D(32, (3, 3), activation='relu'))
        self.model.add(MaxPooling2D(pool_size=(2, 2)))

        # Step 3 - Flattening
        self.model.add(Flatten())

        # Step 4 - Full connection
        self.model.add(Dense(units=128, activation='relu'))
        self.model.add(Dense(units=1, activation='sigmoid'))

        # Compiling the CNN
        self.model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
        return self.model

    def train(self):
        print('Training model: %s' % self.model_name)
        train_datagen = ImageDataGenerator(
            rescale=1. / 255,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True
        )

        test_datagen = ImageDataGenerator(rescale=1. / 255)

        training_set = train_datagen.flow_from_directory(
            self.data_path,
            target_size=(64, 64),
            batch_size=32,
            class_mode='binary'
        )

        test_set = test_datagen.flow_from_directory(
            os.path.join(self.project_path, 'data/test/'),
            target_size=(64, 64),
            batch_size=32,
            class_mode='binary'
        )

        self.model.fit_generator(
            training_set,
            steps_per_epoch=100,
            epochs=10,
            validation_data=test_set,
            validation_steps=2000
        )

    def predict(self, image_array):
        result = self.model.predict_classes(image_array)
        print('The result of prediction is: %s' % str(self.labels.get(result[0][0])))
        return self.labels.get(result[0][0])


class MultiClassDecisionDriverLayer(BaseDriverLayer):

    def build_model(self):
        inp = Input(shape=(200, 200, 3))
        self.model = Conv2D(32, (3, 3), padding='same')(inp)
        self.model = Activation('relu')(self.model)
        self.model = Conv2D(32, (3, 3))(self.model)
        self.model = Activation('relu')(self.model)
        self.model = MaxPooling2D(pool_size=(2, 2))(self.model)
        self.model = Dropout(0.25)(self.model)
        self.model = Conv2D(64, (3, 3), padding='same')(self.model)
        self.model = Activation('relu')(self.model)
        self.model = Conv2D(64, (3, 3))(self.model)
        self.model = Activation('relu')(self.model)
        self.model = MaxPooling2D(pool_size=(2, 2))(self.model)
        self.model = Dropout(0.25)(self.model)
        self.model = Flatten()(self.model)
        self.model = Dense(512)(self.model)
        self.model = Activation('relu')(self.model)
        self.model = Dropout(0.5)(self.model)
        output1 = Dense(1, activation='sigmoid')(self.model)
        output2 = Dense(1, activation='sigmoid')(self.model)
        output3 = Dense(1, activation='sigmoid')(self.model)
        model = Model(inp, [output1, output2, output3])
        model.compile(
            optimizers.rmsprop(lr=0.0001, decay=1e-6),
            loss=[
                "binary_crossentropy",
                "binary_crossentropy",
                "binary_crossentropy",
            ],
            metrics=["accuracy"]
        )
        return model

    def train(self):
        train_datagen = ImageDataGenerator(
            rescale=1. / 255,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True
        )
        valid_datagen = ImageDataGenerator(
            rescale=1. / 255,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True
        )

        test_datagen = ImageDataGenerator(
            rescale=1. / 255,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True
        )

        train_generator = train_datagen.flow_from_directory(
            directory=os.path.join(self.data_path, 'train/'),
            target_size=(200, 200),
            color_mode="rgb",
            batch_size=32,
            class_mode='categorical',
            shuffle=True,
            seed=42
        )

        valid_generator = valid_datagen.flow_from_directory(
            directory=os.path.join(self.data_path, 'valid/'),
            target_size=(200, 200),
            color_mode="rgb",
            batch_size=32,
            class_mode="categorical",
            shuffle=True,
            seed=42
        )

        test_generator = test_datagen.flow_from_directory(
            directory=os.path.join(self.data_path, 'test/'),
            target_size=(200, 200),
            color_mode="rgb",
            batch_size=32,
            class_mode='categorical'
        )

        STEP_SIZE_TRAIN = train_generator.n // train_generator.batch_size
        STEP_SIZE_VALID = valid_generator.n // valid_generator.batch_size

        self.model.fit_generator(
            generator=train_generator,
            steps_per_epoch=STEP_SIZE_TRAIN,
            validation_data=valid_generator,
            validation_steps=STEP_SIZE_VALID,
            epochs=10
        )

        self.model.evaluate_generator(generator=valid_generator, steps=STEP_SIZE_VALID)

        STEP_SIZE_TEST = test_generator.n // test_generator.batch_size
        test_generator.reset()
        pred = self.model.predict_generator(test_generator, steps=STEP_SIZE_TEST, verbose=1)
        predicted_class_indices = np.argmax(pred, axis=1)
        labels = train_generator.class_indices
        self.labels = dict((v, k) for k, v in labels.items())
        predictions = [self.labels[k] for k in predicted_class_indices]
        print(predictions)

    def predict(self, image_data):
        image_array = image.img_to_array(image_data)
        image_array = np.expand_dims(image_array, axis=0)
        result = self.model.predict_classes(image_array)
        return self.labels.get(result[0])


if __name__ == '__main__':
    # PROJECT_PATH = '/home/bot/catkin_ws/ml'
    PROJECT_PATH = '/Users/mehrdad/Documents/Dev/TheGreenBots/'

    # left_or_right_layer = BinaryDecisionDriverLayer(
    #     PROJECT_PATH,
    #     data_path='data/images/',
    #     model_name='left_or_right_layer',
    #     labels={1: 'left', 0: 'right'}
    # )
    #
    # # left_or_right_layer.train()
    #
    # # left_or_right_layer.save_model()
    # left_or_right_layer.load_model()
    #
    # predictions = []
    # print('Testing images with expected prediction: LEFT.')
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/0a9426a3-7e9d-4b5b-9787-87d42f0aafd3.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/f78952ac-20e0-4aac-9051-8f2f758fcac3.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/f881e43d-67ea-46b4-9735-d06e55016a84.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/f8efbc9b-7a68-4a15-9f02-4fe0dca76c5e.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/f97109b0-27c1-47a4-bdac-cb4c43293421.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/f9ff0d2a-f52b-4d73-b5fc-8edf778afe7b.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/fa09e1f8-b4de-4bc0-8d83-f4014cc717b4.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/fb4b2d76-7da4-42dc-a008-ae1369b777f2.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/fc2ab0b8-73f7-44df-9f55-c478cbdbfd48.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/fd5be693-f2c4-4386-8b15-2e8620476e35.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/fdbeb886-c496-49b9-90a4-aa71a4c3a025.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/left/ffb6def2-62db-4cde-b8b2-61c71b328b54.png')))
    # for pred in predictions:
    #     print(pred)
    #
    # print('Testing images with expected prediction: RIGHT.')
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/f77f349f-0d59-4779-b18d-58ea1e41e03f.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/f84670b4-07f9-493a-a7c2-4034bccba907.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/f8f1deed-833b-417f-8de8-24084d336789.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fb15e989-c100-4a88-a89f-eff86fb18de5.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fb41bf00-1969-469a-824a-4972e02936d9.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fb528e8d-21e2-4a8b-89bc-5f9137924c53.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fc6f83a9-d679-49de-9c8d-8d2fa34dc957.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fe000c0a-ec64-4c9b-a6b8-7cddec6b774a.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/fe59a470-6513-455c-a885-808d2c0947fd.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/ff56b49e-d93a-4cfa-9404-6e6bd8900ab1.png')))
    # predictions.append(left_or_right_layer.predict(left_or_right_layer.load_image_from_disk('data/images2/right/ffc3215d-7d9b-4f1d-beed-b668c5a107ed.png')))
    # for pred in predictions:
    #     print(pred)
    #
    # print(left_or_right_layer.model.summary())
    #
    # straight_layer = BinaryDecisionDriverLayer(
    #     PROJECT_PATH,
    #     data_path='data/images/',
    #     model_name='straight_layer',
    #     labels={1: 'straight', 0: 'left_or_right'}
    # )
    #
    # # straight_layer.train()
    #
    # # straight_layer.save_model()
    # straight_layer.load_model()
    #
    # print(straight_layer.model.summary())
    #
    # predictions = []
    # print('Testing LEFT images with expected prediction: left_or_right.')
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/0a9426a3-7e9d-4b5b-9787-87d42f0aafd3.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/f78952ac-20e0-4aac-9051-8f2f758fcac3.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/f881e43d-67ea-46b4-9735-d06e55016a84.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/f8efbc9b-7a68-4a15-9f02-4fe0dca76c5e.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/f97109b0-27c1-47a4-bdac-cb4c43293421.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/f9ff0d2a-f52b-4d73-b5fc-8edf778afe7b.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/fa09e1f8-b4de-4bc0-8d83-f4014cc717b4.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/fb4b2d76-7da4-42dc-a008-ae1369b777f2.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/fc2ab0b8-73f7-44df-9f55-c478cbdbfd48.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/fd5be693-f2c4-4386-8b15-2e8620476e35.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/fdbeb886-c496-49b9-90a4-aa71a4c3a025.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/left/ffb6def2-62db-4cde-b8b2-61c71b328b54.png')))
    # for pred in predictions:
    #     print('Expected left_or_right and got %s' % pred)
    #
    #
    #
    # print('Testing RIGHT images with expected prediction: left_or_right.')
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/f77f349f-0d59-4779-b18d-58ea1e41e03f.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/f84670b4-07f9-493a-a7c2-4034bccba907.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/f8f1deed-833b-417f-8de8-24084d336789.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fb15e989-c100-4a88-a89f-eff86fb18de5.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fb41bf00-1969-469a-824a-4972e02936d9.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fb528e8d-21e2-4a8b-89bc-5f9137924c53.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fc6f83a9-d679-49de-9c8d-8d2fa34dc957.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fe000c0a-ec64-4c9b-a6b8-7cddec6b774a.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/fe59a470-6513-455c-a885-808d2c0947fd.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/ff56b49e-d93a-4cfa-9404-6e6bd8900ab1.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images2/right/ffc3215d-7d9b-4f1d-beed-b668c5a107ed.png')))
    # for pred in predictions:
    #     print('Expected left_or_right and got %s' % pred)
    #
    # predictions = []
    # print('Testing STRAIGHT images with expected prediction: straight.')
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0a7f6cc4-4eed-44a8-945b-f5eae0960a82.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0a709fa6-34ec-4fac-859a-167a0dacca34.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0aafa20b-6019-4f75-a5c8-5fb76721ecb6.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0b58232b-3b0c-4304-97f6-4333317aa2cc.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0bfbe8f3-120e-420e-a214-6c2f343e1e22.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0d29f49c-38d2-4828-81d5-6ec318383c87.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0d603c6d-0dd6-49c7-89e2-f8c106ac0411.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0df11590-0433-4032-8091-a5f41e33384a.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0e8f1b57-5733-46b4-bb4e-b024ea827070.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0e44ca74-4166-499f-955e-2499b2af8e57.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/0e543d32-8dbd-40d0-9b8a-f276679dec89.png')))
    # predictions.append(straight_layer.predict(straight_layer.load_image_from_disk('data/images/straight/00a640d7-7d60-41fd-b6ac-99fe5a3399e5.png')))
    # for pred in predictions:
    #     print('Expected straight and got %s' % pred)


    multi_class_decision_layer = MultiClassDecisionDriverLayer(
        PROJECT_PATH,
        data_path='data/',
        model_name='multi_class_decision_layer',
        labels={
            0: 'left',
            1: 'straight',
            2: 'right'
        }
    )

    multi_class_decision_layer.train()

    multi_class_decision_layer.save_model()
    # multi_class_decision_layer.load_model()
    print('Labels:\n\t', multi_class_decision_layer.labels)
    print(multi_class_decision_layer.model.summary())

    predictions = []
    print('Testing LEFT images with expected prediction: left_or_right.')
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/0a9426a3-7e9d-4b5b-9787-87d42f0aafd3.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/f78952ac-20e0-4aac-9051-8f2f758fcac3.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/f881e43d-67ea-46b4-9735-d06e55016a84.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/f8efbc9b-7a68-4a15-9f02-4fe0dca76c5e.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/f97109b0-27c1-47a4-bdac-cb4c43293421.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/f9ff0d2a-f52b-4d73-b5fc-8edf778afe7b.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/fa09e1f8-b4de-4bc0-8d83-f4014cc717b4.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/fb4b2d76-7da4-42dc-a008-ae1369b777f2.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/fc2ab0b8-73f7-44df-9f55-c478cbdbfd48.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/fd5be693-f2c4-4386-8b15-2e8620476e35.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/fdbeb886-c496-49b9-90a4-aa71a4c3a025.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/left/ffb6def2-62db-4cde-b8b2-61c71b328b54.png')))
    for pred in predictions:
        print('Expected left_or_right and got %s' % pred)



    print('Testing RIGHT images with expected prediction: left_or_right.')
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/f77f349f-0d59-4779-b18d-58ea1e41e03f.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/f84670b4-07f9-493a-a7c2-4034bccba907.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/f8f1deed-833b-417f-8de8-24084d336789.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fb15e989-c100-4a88-a89f-eff86fb18de5.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fb41bf00-1969-469a-824a-4972e02936d9.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fb528e8d-21e2-4a8b-89bc-5f9137924c53.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fc6f83a9-d679-49de-9c8d-8d2fa34dc957.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fe000c0a-ec64-4c9b-a6b8-7cddec6b774a.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/fe59a470-6513-455c-a885-808d2c0947fd.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/ff56b49e-d93a-4cfa-9404-6e6bd8900ab1.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images2/right/ffc3215d-7d9b-4f1d-beed-b668c5a107ed.png')))
    for pred in predictions:
        print('Expected left_or_right and got %s' % pred)

    predictions = []
    print('Testing STRAIGHT images with expected prediction: straight.')
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0a7f6cc4-4eed-44a8-945b-f5eae0960a82.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0a709fa6-34ec-4fac-859a-167a0dacca34.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0aafa20b-6019-4f75-a5c8-5fb76721ecb6.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0b58232b-3b0c-4304-97f6-4333317aa2cc.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0bfbe8f3-120e-420e-a214-6c2f343e1e22.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0d29f49c-38d2-4828-81d5-6ec318383c87.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0d603c6d-0dd6-49c7-89e2-f8c106ac0411.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0df11590-0433-4032-8091-a5f41e33384a.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0e8f1b57-5733-46b4-bb4e-b024ea827070.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0e44ca74-4166-499f-955e-2499b2af8e57.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/0e543d32-8dbd-40d0-9b8a-f276679dec89.png')))
    predictions.append(multi_class_decision_layer.predict(multi_class_decision_layer.load_image_from_disk('data/images/straight/00a640d7-7d60-41fd-b6ac-99fe5a3399e5.png')))
    for pred in predictions:
        print('Expected straight and got %s' % pred)

