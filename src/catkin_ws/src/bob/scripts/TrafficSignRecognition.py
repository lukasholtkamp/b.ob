#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import pandas as pd
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image

# Load the TensorFlow model
MODEL_FILE = "/home/ubuntu/Desktop/FIles for cnn/FIles for raspberry/abcd.h5"
model = load_model(MODEL_FILE)

# Load labels
labels_df = pd.read_csv('/home/ubuntu/Desktop/FIles for cnn/FIles for raspberry/Labels/labelscam.csv')

bridge = CvBridge()

def prediction(img_array, threshold=0.96):
    # Normalize the image array
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)

    # Predict the label
    with tf.device('/CPU:0'):  # Use CPU for prediction
        output = model.predict(img_array)

    pred = np.argmax(output)
    max_prob = np.max(output)

    # Filter out predictions with low confidence scores using a threshold
    if max_prob < threshold:
        pred_str = "None"
        confidence = 0.0
    else:
        pred_str = labels_df[labels_df['ClassId'] == pred]['Name'][pred]
        confidence = max_prob

    return pred_str, confidence

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Resize the image to match the model input size
    resized_frame = cv2.resize(cv_image, (64, 64))

    # Get the predicted label and confidence score for the resized frame
    pred_str, confidence = prediction(resized_frame)

    # Create a message to be published
    result_msg = String()

    # Filter out predictions with low confidence scores using a threshold
    if confidence >= 0.96:
        result_msg.data = pred_str
    else:
        result_msg.data = "ahead"

    # Publish the result on a new topic
    detection_pub.publish(result_msg)

def traffic_sign_recognition_node():
    rospy.init_node('TrafficSignRecognition', anonymous=True)

    # Subscribe to the camera image topic
    rospy.Subscriber('/cam_img', Image, image_callback)

    # Publish the detection result on a new topic
    global detection_pub
    detection_pub = rospy.Publisher('/traffic_sign_detection', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        traffic_sign_recognition_node()
    except rospy.ROSInterruptException:
        pass

