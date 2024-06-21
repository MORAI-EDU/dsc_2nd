#!/usr/bin/env python3
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
        result = cv2.Canny(img_gray, 50, 150)

        # sobel
        sobel_x = cv2.Sobel(img_gray, -1, 1, 0)
        sobel_y = cv2.Sobel(img_gray, -1, 0, 1)
        sobel_xy =  sobel_x + sobel_y


        result = np.concatenate([sobel_xy, result], axis=1)
        cv2.imshow("Sobel vs Canny", result)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        Canny = Canny()
    except rospy.ROSInterruptException:
        pass