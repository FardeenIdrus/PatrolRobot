#!/usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from second_coursework.srv import YOLOlastframe, YOLOlastframeResponse
from second_coursework.msg import YOLODetection
from yolov4 import Detector
import random


class YOLOv4ROSITR:
    def __init__(self):
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k22022028/ros_ws/src/second_coursework/cfg/coco.data')

        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)

        self.yolo_srv = rospy.Service('/detect_frame', YOLOlastframe, self.yolo_service)
        self.pub = rospy.Publisher("/test_image", Image, queue_size=1)
        self.cv_image = None
        self.colors = {}


    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def yolo_service(self, request):
        res = YOLOlastframeResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            cv_height, cv_width, _ = cv_copy.shape
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)

                d.bbox_x = int((d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int((d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int((d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)

            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_copy, encoding="passthrough"))

        return res


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()

