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
    pub = rospy.Publisher('/usb_webcam/qr_data', Int32, queue_size=1)
    rospy.init_node('qr_code_publisher', anonymous=True)
    rate = rospy.Rate(5)  # Adjust publishing rate (e.g., 5 Hz)

    frame_skip = 3  # Process every 3rd frame
    current_frame = 0

    while (cap.isOpened()):
        ret, img = cap.read()

        current_frame += 1

        if current_frame >= frame_skip:
            current_frame = 0  # Reset frame counter

            # Lower image resolution
            img_resized = cv2.resize(img, (640, 480))

            qrDecoder = cv2.QRCodeDetector()
            data, _, _ = qrDecoder.detectAndDecode(img_resized)  # Ignore _, _ values

            if len(data) > 0:
                print("Decoded data: {}".format(data))
                pub.publish(int(data))

            else:
                print("QRCode not detected")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

talker()
