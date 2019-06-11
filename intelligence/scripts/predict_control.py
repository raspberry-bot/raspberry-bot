# TensorFlow and tf.keras
import tensorflow as tf
import keras
from keras import Sequential
from keras.models import model_from_json
from keras.preprocessing import image
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img

# Helper libraries
import numpy as np
import os
import cv2

# Pillow
import PIL
from PIL import Image

import json


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['KERAS_BACKEND'] = 'tensorflow'


PROJECT_PATH = '/Users/mehrdad/Documents/Dev/TheGreenBots/data'
# PROJECT_PATH = '/home/bot/catkin_ws/ml'

def jpeg_to_8_bit_greyscale(path, maxsize):
    img = Image.open(path).convert('L')  # convert image to 8-bit grayscale
    # Make aspect ratio as 1:1, by applying image crop.
    # Please note, croping works for this data set, but in general one
    # needs to locate the subject and then crop or scale accordingly.
    WIDTH, HEIGHT = img.size
    if WIDTH != HEIGHT:
        m_min_d = min(WIDTH, HEIGHT)
        img = img.crop((0, 0, m_min_d, m_min_d))
    # Scale the image to the requested maxsize by Anti-alias sampling.
    img.thumbnail(maxsize, PIL.Image.ANTIALIAS)
    return np.asarray(img)


def get_all_files(path):
    for dirpath, dirnames, filenames in os.walk(path):
        for filename in [f for f in filenames if f.endswith(".png")]:
            yield os.path.join(dirpath, filename)


def load_image_dataset(path_dir, maxsize):
    print('Loading images from %s ...' % path_dir)
    images = []
    new_labels = []
    negative_stats = []
    for img_f in get_all_files(path_dir):
        try:
            img = jpeg_to_8_bit_greyscale(img_f, maxsize)
            images.append(img)
            new_labels.append(img_f.split('/')[-2])
        except Exception as ke:
            negative_stats.append(img_f)
    print('Could not find labels for %d images' % len(negative_stats))
    return (np.asarray(images), np.asarray(new_labels))


def display_images(images, labels, class_names):
    print('Display images...')
    plt.figure(figsize=(10, 10))
    grid_size = min(25, len(images))
    for i in range(grid_size):
        plt.subplot(5, 5, i + 1)
        plt.xticks([])
        plt.yticks([])
        plt.grid(False)
        plt.imshow(images[i], cmap=plt.cm.binary)
        try:
            plt.xlabel(class_names.index(labels[i].tolist()))
        except Exception as ex:
            print(class_names, labels[i])
            pass


def prepare_label_data(path):
    print('Preparing label data...')
    control_labels = {}
    with open(path) as labels_f:
        control_images = json.load(labels_f)
    for item in control_images:
        if type(item['control']) is not list:
            speed, steering = 0.0, 0.0
        else:
            speed, steering = item['control']
        # if not speed:
        #     speed = 'stop'  # 0 # stop
        # if speed > 0:
        #     speed = 'forward'  # 0.26 # forward
        #
        # if not steering:
        #     steering = 'straight'  # 0 # straight
        # if steering > 0:
        #     steering = 'right'  # 2.65 # right
        # elif steering < 0:
        #     steering = 'left'  # -2.65 # left
        control_labels[item['image']] = (speed, steering)
    return control_labels


def save_model(model):
    print('Saving model...')
    # serialize model to JSON
    model_json = model.to_json()
    with open(os.path.join(PROJECT_PATH, "model.json"), "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    model.save_weights(os.path.join(PROJECT_PATH, 'model.h5'))
    print("Saved model to disk")


def load_model():
    # load json and create model
    json_file = open(os.path.join(PROJECT_PATH, 'model.json'), 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)
    # load weights into new model
    loaded_model.load_weights(os.path.join(PROJECT_PATH, 'model.h5'))
    print("Loaded model from disk")
    return loaded_model


def train():
    print('Training started...')
    maxsize = (100, 100)

    # train_labels_data = prepare_label_data('/Users/mehrdad/Documents/Dev/TheGreenBots/data/labels.json')

    (train_images, train_labels) = load_image_dataset(os.path.join(PROJECT_PATH, 'images/'), maxsize)

    # display_images(train_images, train_labels, class_names)
    # plt.show()

    # Setting up the layers.
    classifier = Sequential()
    classifier.add(keras.layers.Conv2D(32, (3, 3), input_shape=(100, 100, 3), activation='relu'))
    classifier.add(keras.layers.MaxPooling2D(pool_size=(2, 2)))
    classifier.add(keras.layers.Flatten())
    classifier.add(keras.layers.Dense(units=128, activation='relu'))
    classifier.add(keras.layers.Dense(units=1, activation='sigmoid'))

    classifier.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

    train_datagen = ImageDataGenerator(
        rescale=1. / 255,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True
    )
    test_datagen = ImageDataGenerator(rescale=1. / 255)
    training_set = train_datagen.flow_from_directory(
        os.path.join(PROJECT_PATH, 'images/'),
        batch_size=32,
        class_mode='binary',
        color_mode='rgb'
    )
    test_set = test_datagen.flow_from_directory(
        os.path.join(PROJECT_PATH, 'straight/'),
        batch_size=32,
        class_mode='binary',
        color_mode='rgb'
    )

    classifier.fit_generator(
        training_set,
        steps_per_epoch=10,  # 8000
        epochs=5,
        validation_data=test_set,
        validation_steps=20  # 2000
    )
    return classifier


def xtrain():
    # Importing the Keras libraries and packages
    from keras.models import Sequential
    from keras.layers import Conv2D
    from keras.layers import MaxPooling2D
    from keras.layers import Flatten
    from keras.layers import Dense

    # Initialising the CNN
    classifier = Sequential()

    # Step 1 - Convolution
    classifier.add(Conv2D(32, (3, 3), input_shape = (64, 64, 3), activation = 'relu'))

    # Step 2 - Pooling
    classifier.add(MaxPooling2D(pool_size = (2, 2)))

    # Adding a second convolutional layer
    classifier.add(Conv2D(32, (3, 3), activation = 'relu'))
    classifier.add(MaxPooling2D(pool_size = (2, 2)))

    # Step 3 - Flattening
    classifier.add(Flatten())

    # Step 4 - Full connection
    classifier.add(Dense(units = 128, activation = 'relu'))
    classifier.add(Dense(units = 1, activation = 'sigmoid'))

    # Compiling the CNN
    classifier.compile(optimizer = 'adam', loss = 'binary_crossentropy', metrics = ['accuracy'])

    # Part 2 - Fitting the CNN to the images

    from keras.preprocessing.image import ImageDataGenerator

    train_datagen = ImageDataGenerator(rescale = 1./255,
                                       shear_range = 0.2,
                                       zoom_range = 0.2,
                                       horizontal_flip = True)

    test_datagen = ImageDataGenerator(rescale = 1./255)

    training_set = train_datagen.flow_from_directory(os.path.join(PROJECT_PATH, 'images/'),
                                                     target_size = (64, 64),
                                                     batch_size = 32,
                                                     class_mode = 'binary')

    test_set = test_datagen.flow_from_directory(os.path.join(PROJECT_PATH, 'straight/'),
                                                target_size = (64, 64),
                                                batch_size = 32,
                                                class_mode = 'binary')

    classifier.fit_generator(training_set,
                             steps_per_epoch = 100,
                             epochs = 10,
                             validation_data = test_set,
                             validation_steps = 2000)
    return classifier


if __name__ == '__main__':
    maxsize = (100, 100)
    class_names = ['left', 'right']

    model = xtrain()
    save_model(model)

    # model = load_model()

    print(model.summary())
    test_image = image.load_img(os.path.join(PROJECT_PATH, 'images/right/3d5b6c7e-060a-4f06-9ec9-f5cb9d28ef13.png'), target_size=(64, 64))
    test_image = image.img_to_array(test_image)
    test_image = np.expand_dims(test_image, axis=0)

    # test_image = cv2.imread(os.path.join(PROJECT_PATH, 'images/left/0a96b6df-4fb3-4e85-b1f3-dc88242962ba.png'))
    # test_image = cv2.resize(img,(100,100))
    # test_image = np.reshape(img,[1,100,100,3])

    # test_image = cv2.resize(
    #     cv2.imread(os.path.join(PROJECT_PATH, 'images/left/0a96b6df-4fb3-4e85-b1f3-dc88242962ba.png')), 
    #     (100, 100)
    # ).astype(np.float32)

    # # test_image = np.expand_dims(test_image, axis=0)

    # print(test_image.shape)
    result = model.predict_classes(test_image)
    print(result)
    if result[0] == 1:
        prediction = 'left'
    else:
        prediction = 'right'

    print(prediction)


    # train_labels = prepare_label_data('/Users/mehrdad/Documents/Dev/TheGreenBots/data/labels.json')
    # (test_images, test_labels) = load_image_dataset('/Users/mehrdad/Documents/Dev/TheGreenBots/data/test/', maxsize, train_labels)
    #
    # print(model.predict(test_images))

    # test_trained_model(model, class_names)

