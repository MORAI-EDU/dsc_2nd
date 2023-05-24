#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os

from sensor_msgs.msg import CompressedImage, Imu

from tf.transformations import euler_from_quaternion
from rotation_translation import IP_matrix
from LUT import LUT

class IPM:
    def __init__(self):
        rospy.init_node('IMP', anonymous=True)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pitch = 0

        self.LUT_lists = dict()
        self.LUT_tables = []
        value = -0.008
        while value < 0.019:
            self.LUT_tables.append(value)
            # input camera params(car > camera) : fov, width, height, roll, pitch, yaw, x, y, z
            rt = IP_matrix(70, 1920, 1080, 
                        np.radians(0), np.radians(5) + value, np.radians(0),
                        1.69, 0, 1.25 + 0.33)
            RT, K = rt.get_ip_matrix()
            # MAKE LUT :  world_max_x, world_min_x, x_interval, world_max_y, world_min_y, y_interval, intrinsic_matrix, extrinsic_matrix)
            TASK = LUT(40, 0, 0.03,
                    11, -11, 0.04,
                    K, RT)
            temp = TASK.get_lut()
            value = round(value,4)
            self.LUT_lists[f"{value}_x"] = temp[0]
            self.LUT_lists[f"{value}_y"] = temp[1]
            value += 0.0001
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)        
        rospy.spin()


    def callback(self, msg):
        self.is_image = True

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        result_target = 10
        current_gap = 10
        for value in self.LUT_tables:
            temp_gap = abs(value - self.pitch)
            if temp_gap < current_gap:
                result_target = value
                current_gap = temp_gap
        result_target = round(result_target,4)
        print(result_target)
        lut_x = self.LUT_lists[f"{result_target}_x"]
        lut_y = self.LUT_lists[f"{result_target}_y"]

        result = cv2.remap(img_bgr, lut_x, lut_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        result = cv2.resize(result, (480, 640))
        cv2.imshow("ipm", result)
        cv2.waitKey(1)


    def imu_callback(self, data):
        quaternion=(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        _, pitch, _ = euler_from_quaternion(quaternion)
        self.pitch = round(pitch,4)


if __name__ == '__main__':
    try:
        IPM = IPM()
    except rospy.ROSInterruptException:
        pass