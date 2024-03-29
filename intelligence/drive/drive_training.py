from keras.models import Sequential
from keras.layers import Dense, Flatten, Dropout, Conv2D, Cropping2D, Lambda
from keras.preprocessing import image
from keras.callbacks import TensorBoard
import csv
import cv2
import numpy as np

# center_images = []
# left_images = []
# right_images = []
images = []
steerings = []
speeds = []
CORRECTION = 0.3


# Adding the center, left, right images together with their steering values
def augment_images(images, measurements, correction=0.0):
    """Augment out training image repository by adding flipped versions with inverted steering."""
    aug_imgs, aug_msrs = [], []
    for image, measurement, in zip(images, measurements):
        corr_msr = measurement + correction
        aug_imgs.append(image)
        aug_msrs.append(corr_msr)
    return aug_imgs, aug_msrs


# Loading the Training data
with open("metadata.csv") as metadata_f:
    metadata = csv.reader(metadata_f, delimiter=',')

    for row in metadata:
        speeds.append(float(row[0]))
        steerings.append(float(row[1]))
        images.append(float(row[2]))
        # speeds.append(float(row[6]))

imgs, control = augment_images(images, steerings, correction=CORRECTION)
# aug_lt_imgs, aug_lf_msrs = augment_images(left_images, steerings, correction=CORRECTION)
# aug_rt_imgs, aug_rt_msrs = augment_images(right_images, steerings, correction=CORRECTION * -1)
# X_train = np.array(aug_cr_imgs + aug_lt_imgs + aug_rt_imgs)
# y_train = np.array(aug_cr_msrs + aug_lf_msrs + aug_rt_msrs)

X_train = np.array(imgs)
Y_train = np.array(control)

Epochs = 10
# Batch_size = 32

# Neural Network Archticture

model = Sequential()
model.add(Cropping2D(cropping=((200, 200), (0, 0))))
model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160, 225, 3)))
model.add(Conv2D(24, (5, 5), strides=(2, 2), activation="relu"))
model.add(Conv2D(36, (5, 5), strides=(2, 2), activation="relu"))
model.add(Conv2D(48, (5, 5), strides=(2, 2), activation="relu"))
model.add(Conv2D(64, (3, 3), strides=(2, 2), activation="relu"))
model.add(Flatten())
model.add(Dropout(0.3))
model.add(Dense(100))
model.add(Dropout(0.3))
model.add(Dense(50))
model.add(Dropout(0.2))
model.add(Dense(1))

# Compiling the network with mse loss function and the adam optimizer (No accuracy matrix because it's a regression problem)
model.compile(loss='mean_squared_error', optimizer='adam')

# For monitiring the training in tensorboard
# tbCallBack = TensorBoard(log_dir='./Graph', histogram_freq=0, write_graph=True, write_images=True)

# Training
model.fit(X_train, Y_train, validation_split=0.2, shuffle=True, epochs=Epochs)

# Saving model
model.save('meta_vision.h5')

# Training summary
model.summary()

print('Testing left...')
model.predict()