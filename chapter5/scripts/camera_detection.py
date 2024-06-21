#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloNode:
    def __init__(self):
        rospy.init_node('camera_detection', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback)
        
        rospy.spin()

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            results = self.model(cv_image)[0]

            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                conf = box.conf[0]
                cls = box.cls[0].item() 
                label = f'{self.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the output image
            cv2.imshow("YOLOv8 Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        YoloNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()