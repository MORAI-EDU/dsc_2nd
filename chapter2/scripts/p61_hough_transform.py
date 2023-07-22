#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os, cv2
import numpy as np

from sensor_msgs.msg import CompressedImage


class Canny:
    def __init__(self):
        rospy.init_node('Canny', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        img_w, img_h = img_gray.shape
        canny = cv2.Canny(img_gray, 50, 150)
        lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 100, minLineLength=15, maxLineGap=10)
        img_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_color, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.imshow("Canny", canny)
        cv2.imshow("Image window", img_color)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        Canny = Canny()
    except rospy.ROSInterruptException:
        pass