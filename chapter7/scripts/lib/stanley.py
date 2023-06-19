#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt, atan2, pi, cos, sin

from geometry_msgs.msg import Point

class Stanley_alg():
    def __init__(self, gain_kv,gain_ky, wheelbase, max_steer): # (1.0,0.7, 2.7, 36)
        
        self.current_postion = Point()
        self.gain_kv = gain_kv
        self.gain_ky = gain_ky
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self._path = []

    def get_path(self, path):
        self._path = path

    def get_vehicle_state(self, msg):

        self.current_vel = msg.velocity #kph
        self.vehicle_yaw = (msg.heading) / 180*pi   # rad
        self.current_postion.x = msg.position.x ## 차량의 현재x 좌표 ##
        self.current_postion.y = msg.position.y ## 차량의 현재y 좌표 ##
        self.current_postion.z = msg.position.z ## 차량의 현재z 좌표 ##


        self.front_x = self.current_postion.x + self.wheelbase * cos(self.vehicle_yaw)
        self.front_y = self.current_postion.y + self.wheelbase * sin(self.vehicle_yaw)

    def calculate_steering_angle(self,gear_status):

        gear = gear_status # 2 : R 4 : D

        steering_angle = 0.
        current_waypoint, ey = self.calc_min_path_point(self.front_x,self.front_y)
        if gear == 2:
            current_waypoint, ey_front = self.calc_min_path_point(self.front_x,self.front_y)
            w__ , ey_rear = self.calc_min_path_point(self.front_x,self.front_y)
            ey = ey_front - ey_rear
        
        path_x = self._path.poses[current_waypoint].pose.position.x
        path_y = self._path.poses[current_waypoint].pose.position.y
        pos_path_x = self._path.poses[current_waypoint + 1].pose.position.x
        pos_path_y = self._path.poses[current_waypoint + 1].pose.position.y
        path_yaw = atan2(pos_path_y - path_y, pos_path_x - path_x)

        if gear == 2:
            path_yaw = atan2(path_y - pos_path_y, path_x - pos_path_x)
            yaw_term =  - self.normalize_angle(path_yaw - self.vehicle_yaw) * 3
        else: 
            path_yaw = atan2(pos_path_y - path_y, pos_path_x - path_x)
            yaw_term =  self.normalize_angle(path_yaw - self.vehicle_yaw)

        local_x , local_y = self.local_of_me(path_x, path_y) 
        if(local_y > 0): 
            cte = ey
        else:
            cte = - ey

        cte_term = atan2(self.gain_ky * cte , self.gain_kv + self.current_vel.x)        
        steering_angle = (yaw_term + cte_term)
        return steering_angle

    def local_of_me(self, path_x, path_y):
        theta=self.vehicle_yaw
        translation=[self.front_x, self.front_y]
        rotation = np.array([[cos(theta), -sin(theta), 0], \
                             [sin(theta), cos(theta) , 0], \
                             [0 , 0, 1]])
        trans_rotat = np.array([[rotation[0][0], rotation[1][0], -(rotation[0][0]*translation[0] + rotation[1][0]* translation[1])], \
                                [rotation[0][1], rotation[1][1], -(rotation[0][1]*translation[0] + rotation[1][1]* translation[1])], \
                                [0,0, 1]])

        global_xy = np.array([[path_x], [path_y], [1]])
        local_xy = trans_rotat.dot(global_xy)
        
        local_path_x = local_xy[0][0]
        local_path_y = local_xy[1][0]

        return local_path_x , local_path_y

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2.0 * pi

        while angle < -np.pi:
            angle += 2.0 * pi

        return angle

    def calc_min_path_point(self,point_x,point_y):

        min_dis=float('inf')
        
        for i,pos in enumerate(self._path.poses):
            dx = point_x - self._path.poses[i - 1].pose.position.x
            dy = point_y - self._path.poses[i - 1].pose.position.y
            dis = sqrt(pow(dx,2) + pow(dy,2))
            if dis < min_dis :
                min_dis=dis
                current_waypoint = i

        return current_waypoint, min_dis