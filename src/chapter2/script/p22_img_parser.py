#!/usr/bin/env python
 
import rospy, cv2, os
import numpy as np
from sensor_msgs.msg import CompressedImage


class IMGParser:
    def __init__(self):
        rospy.init_node('IMGParser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        rospy.spin()


    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass