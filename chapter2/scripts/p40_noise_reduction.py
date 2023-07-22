#!/usr/bin/env python
 
import rospy, os, cv2
import numpy as np

from sensor_msgs.msg import CompressedImage


class Gaussian_filtering:
    def __init__(self):
        rospy.init_node('Gaussian_filtering', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()


    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        gaussian_filtering_5X5_1 = cv2.GaussianBlur(img_gray, (5,5), 1)

        result = np.concatenate([img_gray, gaussian_filtering_5X5_1], axis=1)
        cv2.imshow("Gaussian Filtering", result)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        Gaussian_filtering = Gaussian_filtering()
    except rospy.ROSInterruptException:
        pass