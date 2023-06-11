#!/usr/bin/env python
 
import rospy, sys, cv2, numpy as np
from p36_IP_MATRIX import extrinsic_matrix, intrinsic_matrix
from sensor_msgs.msg import CompressedImage, Imu

from tf.transformations import euler_from_quaternion


class Perspective_matrix_test:
    def __init__(self):
        rospy.init_node('Perspective_matrix_test', anonymous=True)

        # IMU sersor
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pitch = None

        # wait for imu connection
        while self.pitch is None: 
            pass
        print(f"imu topic now connected pitch : {self.pitch}")

        # camera extrinsic parameter (vehicle -> camera)
        roll = 0       # degree
        pitch = 10     # degree
        yaw = 0        # degree
        x = 1.77       # m
        y = 0          # m
        z = 1.18       # m

        # camera intrinsic parameter
        fov = 90       # degree
        width = 1920   # pixel
        height = 1080  # pixel

        # Intrinsic(K) Extrinsic(RT) Matrix
        self.K = intrinsic_matrix(fov, width, height)
        self.RT = extrinsic_matrix(np.radians(roll), 
                                   np.radians(pitch) + self.pitch, 
                                   np.radians(yaw), 
                                   x, y, z + 0.3)

        # start processing
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        print("Camera topic now connected")
        
        rospy.spin()


    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        for x,y,z in [(20, -3, 0), (8, -3, 0), (8, 3, 0), (20, 3, 0)]:
            world_xyz = np.array([x, y, z, 1])
            image_xyz = self.K @ self.RT @ world_xyz
            image_xyz /= image_xyz[2]
            image_x, image_y = int(image_xyz[0]), int(image_xyz[1])

            img_bgr = cv2.circle(img_bgr, center=(image_x, image_y), radius=3, color=(0,0,255))
            img_bgr = cv2.putText(img_bgr, f" ({image_x},{image_y})",(image_x, image_y), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1, cv2.LINE_AA)


        img_bgr = cv2.resize(img_bgr, (1920 //2, 1080 //2))
        cv2.imshow("result", img_bgr)
        cv2.waitKey(1)


    def imu_callback(self, data):
        quaternion=(data.orientation.x, 
                    data.orientation.y, 
                    data.orientation.z, 
                    data.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.pitch = pitch


if __name__ == '__main__':
    try:
        perspective_matrix_test = Perspective_matrix_test()
    except rospy.ROSInterruptException:
        pass
