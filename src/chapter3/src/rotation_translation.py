#!/usr/bin/env python
import rospy
import cv2
import numpy as np


class IP_matrix:
    def __init__(self, fov, width, height, roll, pitch, yaw, x, y, z) -> None:
        self.fov = fov
        self.width = width
        self.height = height
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z


    def rt_matrix(self):
        si, sj, sk = np.sin(self.roll), np.sin(self.pitch), np.sin(self.yaw)
        ci, cj, ck = np.cos(self.roll), np.cos(self.pitch), np.cos(self.yaw)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk
        
        R = np.array([[cj * ck, sj * sc - cs, sj * cc + ss, self.x], 
                      [cj * sk, sj * ss + cc, sj * cs - sc, self.y], 
                      [-sj    , cj * si     ,      cj * ci, self.z],
                      [0      , 0           , 0           , 1]])
        return R


    def get_ip_matrix(self):
        fx = fy = self.width / (2*np.tan(np.deg2rad(self.fov/2)))
        cx = self.width / 2
        cy = self.height / 2

        # intrinsic matrix
        K = np.array([[fx, 0 , cx, 0],
                      [0 , fy, cy, 0],
                      [0 , 0 , 1 , 0],
                      [0 , 0 , 0 , 1]])
        
        # extrinsic matrix
        RT_car2cam = self.rt_matrix()

        # coordinates rotation : car, world  >  camera
        R = np.array([[0, -1,  0,  0],
                      [0,  0,  1,  0],
                      [1,  0,  0,  0],
                      [0,  0,  0,  1]])
        
        RT = R @ RT_car2cam

        return RT, K