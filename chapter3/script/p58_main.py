#!/usr/bin/env python
 
import rospy, sys, cv2, os, numpy as np
from tf.transformations import euler_from_quaternion
from IP_MATRIX import extrinsic_matrix, intrinsic_matrix
from LUT import get_lut

from sensor_msgs.msg import CompressedImage, Imu


class IPM:
    def __init__(self, fov, width, height, roll, pitch, yaw, x, y, z):
        rospy.init_node('IPM', anonymous=True)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pitch = None

        # wait for imu connection
        while self.pitch is None:
            pass

        K = intrinsic_matrix(fov, width, height)
        RT = extrinsic_matrix(np.radians(roll), np.radians(pitch) + self.pitch, np.radians(yaw), x, y, z + 0.3)

        # MAKE LUT
        temp = get_lut(35, 0, 0.05, 15, -15, 0.05, K, RT)
        self.lut_x = temp[0]
        self.lut_y = temp[1]

        # start processing
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        
        rospy.spin()


    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        result = cv2.remap(img_bgr, self.lut_x, self.lut_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow("ipm", result)
        cv2.waitKey(1)


    def imu_callback(self, data):
        quaternion=(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        _, pitch, _ = euler_from_quaternion(quaternion)
        self.pitch = round(pitch,6)


if __name__ == '__main__':
    try:
        if len(sys.argv) < 10:
            print(f"""
            --------------------------------------------------------------
              need 9 parameters...                                       
              fov(hirizontal), width, height, roll, pitch, yaw, x, y, z  
              you only give {len(sys.argv)} parameters                   
            --------------------------------------------------------------
            """)
        else:
            args = list(map(float, sys.argv[1:]))
            IPM = IPM(*args)
    except rospy.ROSInterruptException:
        pass