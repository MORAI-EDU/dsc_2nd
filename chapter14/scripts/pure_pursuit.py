#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
import numpy as np

# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)        
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback) 

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2         


        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)        
        self.event = EventInfo()

        self.is_event = False      
        self.is_lattice_path = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78

        # self.plot()
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_global_path and self.is_lattice_path and self.is_status:

                steering = self.calc_pure_pursuit()

                if self.is_look_forward_point :
                    if not self.is_event:
                        self.event.option = 1
                        self.event.ctrl_mode = 3
                        self.event_srv(self.event)

                        self.event.option = 2
                        self.event.gear = 4
                        self.event_srv(self.event)                 


                        self.is_event = True
                    self.ctrl_cmd_msg.steering = steering
                    self.ctrl_cmd_msg.velocity = 60 #target_velocity
                    rospy.loginfo("steering : {0}".format(steering))

                else : 
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                    rospy.loginfo("no found forward point")
                                    
                
                # self.x.append(self.data_count)
                # self.y.append(self.ctrl_cmd_msg.steering * 13.9 * 180 / 3.14)


                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    # def init(self):
    #     self.line.set_data([], [])
    #     return self.line,    
    
    # def update(self, frame):                
    #     self.line.set_data(self.x, self.y)
    #     return self.line,

    # def plot(self):             
    #     self.data_count = 0
    #     self.x = []
    #     self.y = [] 

    #     self.fig, self.ax = plt.subplots()
    #     self.line, = self.ax.plot([], [], lw=2)
    #     self.ax.set_xlim(0, 10)
    #     self.ax.set_ylim(-2, 2)
    #     self.ax.set_xlabel('X(time)')
    #     self.ax.set_ylabel('Y(Wheel angle / deg)')
    #     self.ax.set_title('Wheel angle plot')

    #     ani = FuncAnimation(self.fig, self.update, frames=range(200), init_func=self.init, blit=True, interval=100)

    #     # 애니메이션 실행
    #     plt.show()
        

    def path_callback(self,msg):
        self.is_lattice_path = True
        self.path = msg  

    def status_callback(self,msg):
        self.is_status = True
        self.status_msg = msg    
        self.vehicle_yaw = msg.heading * np.pi / 180
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self,):
        
        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain
        
        if self.lfd < self.min_lfd : 
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd = self.max_lfd
        
        vehicle_position = self.status_msg.position
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw)  , translation[0]],
                                 [sin(self.vehicle_yaw), cos(self.vehicle_yaw)   , translation[1]],
                                 [0                    , 0                       , 1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for path in self.path.poses :
            path_point = path.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0 :
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)

        return steering

if __name__ == '__main__':
    try:
        pure_pursuit()
    except rospy.ROSInterruptException:
        pass
