#!/usr/bin/env python
 
import rospy, cv2, os
import numpy as np
from sensor_msgs.msg import CompressedImage


class Sobel:
    def __init__(self):
        rospy.init_node('Sobel', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        rospy.spin()


    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        sobel_x = cv2.Sobel(img_gray, -1, 1, 0)
        sobel_y = cv2.Sobel(img_gray, -1, 0, 1)
        sobel_xy =  sobel_x + sobel_y

        cv2.imshow('Sobel', sobel_xy)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        Sobel = Sobel()
    except rospy.ROSInterruptException:
        pass