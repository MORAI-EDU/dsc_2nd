#!/usr/bin/env python3
import rospy
import cv2
import numpy as np


def extrinsic_matrix(roll, pitch, yaw, x, y, z):
    si, sj, sk = np.sin(roll), np.sin(pitch), np.sin(yaw)
    ci, cj, ck = np.cos(roll), np.cos(pitch), np.cos(yaw)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk
        
    RT = np.array([[cj * ck,   sj * sc - cs,   sj * cc + ss,   x], 
                   [cj * sk,   sj * ss + cc,   sj * cs - sc,   y], 
                   [-sj    ,   cj * si     ,        cj * ci,   z],])
    
    # coordinates rotation
    R = np.array([[0,  -1,   0],
                  [0,   0,   1],
                  [1,   0,   0],])

    return R @ RT



def intrinsic_matrix(fov_horizontal, width, height):
    fx = fy = width / (2 * np.tan(np.deg2rad(fov_horizontal/2)))
    cx = width / 2
    cy = height / 2

    K = np.array([[fx, 0 , cx],
                  [0 , fy, cy],
                  [0 , 0 , 1 ],])
        
    return K
