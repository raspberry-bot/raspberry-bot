#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""


# Python libs
import sys, time, os

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import json


# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from uuid import uuid4
import requests


from keras.preprocessing import image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = True
TRAINING = True

global global_training_matrix

global_training_matrix = []

from predict_control import load_model

# model = load_model()


def model_snapshot(data):
    # data = []
    # try:
    #     with open('/home/bot/catkin_ws/data/controls.json', "r") as jsonFile:
    #         data = json.load(jsonFile)
    # except Exception as e:
    #     pass
    
    # data += new_rows

    with open('/home/bot/catkin_ws/data/controls.json', "w") as jsonFile:
        json.dump(data, jsonFile)


def move(speed, steer):
    # Starts a new node
    rospy.init_node('Move', anonymous=True)
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = speed
    vel_msg.angular.z = steer
    publisher.publish(vel_msg)


def img_callback(ros_data):
    from keras.preprocessing import image
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    # if VERBOSE :
        # print('received image of type: "%d" %s' % (len(ros_data.data), ros_data.header))

    url = 'http://192.168.2.86:8888/predict-control'
    files = {'image': ros_data.data}
    response = requests.post(url, files=files)
    print(response.content)
    control = -1

    #### direct conversion to CV2 ####
    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    #
    #
    # image_id = str(uuid4())
    # image_name = os.path.join('/home/bot/catkin_ws/data/images/', image_id + '.png')
    # cv2.imwrite(image_name, image_np)
    #
    # print('Storing %s image ...' % image_name)
    #
    # # image = image.load_img(image_name, target_size=(64, 64))
    #
    # test_image = image.load_img(image_name, target_size=(64, 64))
    # test_image = image.img_to_array(test_image)
    # test_image = np.expand_dims(test_image, axis=0)


    # test_image = cv2.resize(cv2.imread(image_name), (64, 64)).astype(np.float32)


    # control = model.predict_classes(test_image)[0]
    #
    control = 'right' if control == 1 else 'left'
    #
    # print(control)
    if control == 'right':
        move(0.27, 1.8)
    elif control == 'left':
        move(0.27, -1.8)

    # if TRAINING:
    #     print(global_training_matrix)
    #     global_training_matrix.append({
    #         'image': image_id,
    #         'control': None,
    #         # 'depth': None,
    #     })
    #     model_snapshot(global_training_matrix)
    
    #### Feature detectors using CV2 #### 
    # "","Grid","Pyramid" + 
    # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
    # method = "GridFAST"
    # feat_det = cv2.FeatureDetector_create(method)
    # time1 = time.time()

    # convert np image to grayscale
    # featPoints = feat_det.detect(cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
    # time2 = time.time()
    # if VERBOSE :
    #     print('%s detector found: %s points in: %s sec.'%(method, len(featPoints),time2-time1))

    # for featpoint in featPoints:
    #     x,y = featpoint.pt
    #     cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
    
    # cv2.imshow('cv_img', image_np)
    # cv2.waitKey(2)

    #### Create CompressedIamge ####
    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    # Publish new image
    # self.image_pub.publish(msg)
    
    #self.subscriber.unregister()

# def control_callback(ros_data):
#     if TRAINING:
#         global_training_matrix[len(global_training_matrix) - 1]['control'] = (ros_data.linear.x, ros_data.angular.z)
#         if all(global_training_matrix[len(global_training_matrix) - 1].values()):
#             model_snapshot(global_training_matrix)

def depth_callback(ros_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    # if VERBOSE :
        # print('received image of type: "%d" %s' % (len(ros_data.data), ros_data.header))

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    if VERBOSE:
        print(np_arr)

    images[rospy.get_rostime()] = np_arr

    if TRAINING:
        global_training_matrix[len(global_training_matrix) - 1]['depth'] = np_arr
        if all(global_training_matrix[len(global_training_matrix) - 1].values()):
            model_snapshot(global_training_matrix)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('StayOnGrassClient', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, img_callback)
    # rospy.Subscriber("/camera/rgb/image_raw/compressedDepth", CompressedImage, depth_callback)
    # rospy.Subscriber("/cmd_vel", Twist, control_callback)
    model_snapshot(global_training_matrix)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
        print()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)