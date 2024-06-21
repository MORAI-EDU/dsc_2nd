#!/usr/bin/env python3
 
import rospy, cv2, os
import numpy as np
from sensor_msgs.msg import CompressedImage


class Basic_filtering:
    def __init__(self):
        rospy.init_node('Basic_filtering', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        rospy.spin()


    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        mask_x = np.array([[ 0, 0, 0], 
                           [-1, 0, 1], 
                           [ 0, 0, 0]])
        mask_y = np.array([[ 0,-1, 0], 
                           [ 0, 0, 0], 
                           [ 0, 1, 0]])

        filtered_x = cv2.filter2D(img_gray, -1, mask_x)
        filtered_y = cv2.filter2D(img_gray, -1, mask_y)

        # cv2.imshow('original', img_gray)
        # cv2.imshow('x', filtered_x)
        # cv2.imshow('y', filtered_y)
        temp = cv2.hconcat([img_gray, filtered_x, filtered_y])
        cv2.imshow('original, x, y', temp)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        Basic_filtering = Basic_filtering()
    except rospy.ROSInterruptException:
        pass