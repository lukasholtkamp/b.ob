#!/usr/bin/env python
import os
import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

from models.experimental import attempt_load
from utils.general import (
    check_img_size,
    check_imshow,
    non_max_suppression,
    scale_coords,
    set_logging,
)
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, time_synchronized

bridge = CvBridge()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        # YOLOv5 configuration
        weights = "yolov5s.pt"  # model.pt path(s)
        self.imgsz = 640  # inference size (pixels)
        self.conf_thres = 0.25  # confidence threshold
        self.iou_thres = 0.45  # NMS IOU threshold
        self.max_det = 1000  # maximum detections per image
        self.classes = None  # filter by class
        self.agnostic_nms = False  # class-agnostic NMS
        self.line_thickness = 3  # bounding box thickness (pixels)
        self.half = False  # use FP16 half-precision inference

        # Initialize YOLOv5 model
        set_logging()
        self.device = select_device("")
        self.half &= self.device.type != "cpu"  # half precision only supported on CUDA
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        self.names = (
            self.model.module.names
            if hasattr(self.model, "module")
            else self.model.names
        )  # get class names
        if self.half:
            self.model.half()  # to FP16

        # Dataloader
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        if self.device.type != "cpu":
            self.model(
                torch.zeros(1, 3, imgsz, imgsz)
                .to(self.device)
                .type_as(next(self.model.parameters()))
            )  # run once

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Keep track of previous detection count
        self.previous_person_count = 0

    def camera_callback(self, data):
        t0 = time.time()
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # Preprocessing
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]  # Add batch dimension
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        # Convert to tensor
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # Normalize to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img)[0]
        pred = non_max_suppression(
            pred,
            self.conf_thres,
            self.iou_thres,
            self.classes,
            self.agnostic_nms,
            max_det=self.max_det,
        )

        person_count = 0  # Reset person count for this frame

        # Process detections
        for det in pred:
            if len(det):
                # Rescale boxes from img_size to original size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                for *xyxy, conf, cls in det:
                    if self.names[int(cls)] == "person":
                        person_count += 1

                    # Draw bounding boxes
                    label = f"{self.names[int(cls)]} {conf:.2f}"
                    plot_one_box(
                        xyxy,
                        img0,
                        label=label,
                        color=colors(int(cls), True),
                        line_thickness=self.line_thickness,
                    )

        # Publish detection message only if the number of persons changes
        if person_count > 0 and person_count != self.previous_person_count:
            if person_count == 1:
                self.get_logger().info(f"One person detected")
            else:
                for i in range(1, person_count + 1):
                    self.get_logger().info(f"Person {i} detected")

        # Update the previous detection count
        self.previous_person_count = person_count

        # Display image with bounding boxes
        cv2.imshow("IMAGE", img0)
        cv2.waitKey(4)


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
