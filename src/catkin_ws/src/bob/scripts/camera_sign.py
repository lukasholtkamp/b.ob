#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")

bridge = CvBridge()

def talker():
    rospy.init_node('camera_node', anonymous=True)
    pub = rospy.Publisher('/cam_img', Image, queue_size=60)
    rate = rospy.Rate(10)  # Adjust publishing rate (e.g., 5 Hz)

    frame_skip = 3  # Process every 3rd frame
    current_frame = 0

    while not rospy.is_shutdown():
        returnValue, img = cap.read()
        if returnValue == True:
            rospy.loginfo('Video frame captured and published')
            current_frame += 1
            ImageToTransmit = bridge.cv2_to_imgmsg(img)
            pub.publish(ImageToTransmit)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

talker()

