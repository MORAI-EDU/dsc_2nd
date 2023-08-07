#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from queue import Queue
from threading import Thread
import xml.etree.ElementTree as ET
import cv2
import rospkg

class RosManager:
    def __init__(self):
        rospy.init_node("morai_standard3", anonymous=True)
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path("chapter5")
        load_path = self.file_path + "/../../yolov8n.pt"
        print(load_path)
        # self.model = YOLO('yolov8n.pt')
        self.model = YOLO(load_path)
        self._set_protocol()
        
        self.ros_rate = rospy.Rate(10)

    def execute(self):
        print("start simulation")
        
        while not rospy.is_shutdown():
            self.ros_rate.sleep()
        
    def image_callback(self, msg):
        try:
            # print("here")
            self.is_img = True
            np_arr = np.fromstring(msg.data, np.uint8)      
            img_color = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)
            
            re = self.model.predict(source=img_color, conf=0.45, show=True)
            cv2.imshow("src_img", img_color)
            cv2.waitKey(1)
        except cv2.error as e:
            print(e)

    def _set_protocol(self):
        # 카메라 이미지 불러오기
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
