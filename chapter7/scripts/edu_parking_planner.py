#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from nav_msgs.msg import Path

from math import cos, sin, sqrt, pow, atan2, tan, pi, atan, radians, degrees
from enum import Enum, IntEnum
from lib.stanley import Stanley_alg
from lib.utils import findLocalPath, pathReader

class GearChange(IntEnum):
    NONE = -1 
    PARK = 1
    REVERSE = 2
    NEUTRAL = 3
    DRIVE = 4

class ParallelParkingStatus(Enum):
    """   
        평행 주차 들어가는 단계
        - 차량 모델 정의 
        - 주차 모델 정의 및 Goal Point, Start Point 생성
        Phase_1 : 현재 위치 부터 Parking Space 와 인접한 Link 까지의 최단 거리 경로 생성        
        Phase_2 : Parking Start Point 에 도달 하였으면 Parking Start Point 에서 주차 공간 까지 3차 곡선으로 경로를 생성해서 후진으로 들어간다
        Phase_3 : 후진 을 통해 주차 공간 진입 후 일정 이상 들어 왔으면 다시 전진 경로 생성 후 진행
        Phase_4 : 주차 공간에 들어 왔다면 성공
    """
    Idle = 0            # 주차 Action 이 없거나 주차 상태에서 완전히 빠져 나온다면 Idle 상태
    searching = 1       # 주차 공간의 Idx 지정 시 주차 공간이 있는 Link 로 이동 중인 상태
    IN_Phase_1 = 2      # 주차 공간이 있는 Link 로 이동 완료 후 주차 를 위해 주차 공간 옆에 정렬 중인 상태
    IN_Phase_2 = 3      # 주차 공간에 진입 중인 상태 이때는 R 단으로 후진이다
    IN_Phase_3 = 4      # 주차 공간에 완전히 진입 하였고 중앙 정렬를 하는 단계 D 단으로 전진한다
    IN_Phase_4 = 5      # 주차 공간에 완전히 진입 하였고 중앙 정렬를 하는 단계 R 단으로 전진한다
    
    parking = 8         # 주차가 완료 된 상태
    waiting = 9         # D --> R R --> D 로 기어 변경 시 속도가 0 이어야 한다 차량 속도가 0 일때 까지 대기중인 상태를 정의

class parallel_parking():
    def __init__(self):
        self.wheelbase = 2.7
        self.max_steer = 36.3
        self.front_overhang = 0.87
        self.rear_overhang = 0.785
        self.vehicleWidth = 1.805

        self.parking_status = ParallelParkingStatus.Idle
        self.next_parking_status = ParallelParkingStatus.Idle

        self.target_vel = 5

        # Overhang 정보를 고려해서 주차 공간 끝 라인에 안전 거리 확보를 위한 값
        self.safety_offset = self.rear_overhang

        # reference angle 주차가 가능한 각도
        self.reference_angle = self.max_steer * 4/5 # * 7/9 # deg

        self._tick_distance = 0.1 # m

        self.vehicle_rear_position = Point()
        self.vehicle_front_position = Point()
        self.vehicle_yaw = 0.0

        self.goal_point = Point()
        self.parking_start_point = Point()
        self.searching_start_point = Point()
        self.searching_end_point = Point()
        self.end_point_1 = Point()
        self.end_point_2 = Point()

        self.C_1 = Point()
        self.C_2 = Point()

        self.parking_space = {
            'point_a' : {
                'x' : 2.651663,
                'y' : 1095.602417,
                'z' : -0.687296
            },
            'point_b' : {
                'x' : 0.323886,
                'y' : 1091.407104,
                'z' : -0.694371
            },
            'point_c' : {
                'x' : 0.13121,
                'y' : 1096.938354,
                'z' : -0.733993
            },
            'point_d' : {
                'x' : -2.161301,
                'y' : 1092.790161,
                'z' : -0.725765
            },
            'width' : 2.8529969511001148
        }

        self.point_A = Point(self.parking_space['point_a']['x'], self.parking_space['point_a']['y'], 0)
        self.point_B = Point(self.parking_space['point_b']['x'], self.parking_space['point_b']['y'], 0)
        self.point_C = Point(self.parking_space['point_c']['x'], self.parking_space['point_c']['y'], 0)
        self.point_D = Point(self.parking_space['point_d']['x'], self.parking_space['point_d']['y'], 0)

        self.get_parking_space()


    def is_in_parking_status(self):
        return self.parking_status not in [ParallelParkingStatus.Idle, ParallelParkingStatus.searching]

    def get_parking_space(self):

        # Parking Space Perpendicular_heading, parallel_heading
        self.parallel_heading = atan2(self.point_B.y - self.point_A.y, self.point_B.x - self.point_A.x) #Parking space heading 
        self.Perpendicular_heading = atan2(self.point_C.y - self.point_A.y, self.point_C.x - self.point_A.x) #Parking space perpendicular heading

        # length, width
        dis = self.parking_space['width'] / 2 

        goal_x,goal_y = self.rotation_matrix_inv(self.parallel_heading, self.point_A.x, self.point_A.y, self.safety_offset, -dis) 
        self.goal_point = Point(goal_x, goal_y, 0) # Point_E

        end_1_x,end_1_y = self.rotation_matrix_inv(self.parallel_heading,self.point_A.x, self.point_A.y, 0, -dis)
        end_2_x,end_2_y = self.rotation_matrix_inv(self.parallel_heading,self.point_B.x, self.point_B.y, 0, -dis)

        self.end_point_1 = Point(end_1_x, end_1_y, 0) # the center of point a and point c
        self.end_point_2 = Point(end_2_x, end_2_y, 0) # the center of point b and point d

        min_r = self.wheelbase / tan(self.max_steer * pi/180) # Re

        y_dis = min_r  # parking_space_y_offset
        x_dis = y_dis/tan(self.reference_angle*(pi/180)) # parking_space_x_offset

        parking_start_point_x,parking_start_point_y = self.rotation_matrix_inv(self.parallel_heading,self.goal_point.x,self.goal_point.y, x_dis, y_dis)
        self.parking_start_point = Point(parking_start_point_x, parking_start_point_y, 0) # Einit        

    def rotation_matrix_inv(self,theta__,tmp_x,tmp_y,local_p_x,local_p_y):

        tmp_theta = theta__ # rad
        tmp_translation=[tmp_x,tmp_y]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        local_result=np.array([[local_p_x],[local_p_y],[1]])

        tmp_det_t = np.linalg.inv(tmp_det_t)

        global_result=tmp_det_t.dot(local_result)

        return global_result[0][0],global_result[1][0]  # x,y

    def rotation_matrix(self, x_1,x_2,y_1,y_2):

        translation=[x_1,y_1]
        theta=atan2(y_2-y_1,x_2-x_1)

        t=np.array([        [cos(theta), -sin(theta),translation[0]],   \
                            [sin(theta),cos(theta),translation[1]],     \
                            [0,0,1]])
        det_t=np.array([    [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])   ],\
                            [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])   ],\
                            [0,0,1]])

        return t, det_t
        
    def changecoordinate_for_lc(self,start_point,end_point,rotation_point):
        # 지역 좌표계로 변환
        t, det_t = self.rotation_matrix(start_point.x,rotation_point.x,start_point.y,rotation_point.y)
        
        world_end_point=np.array([[end_point.x],[end_point.y],[1]])
        local_end_point=det_t.dot(world_end_point)

        x_start=0
        x_end=local_end_point[0][0]

        y_start=0.0
        y_end=local_end_point[1][0]

        return  x_start,x_end,y_start,y_end, t  

    def circle_arc_point(self,point_1_x,point_1_y,point_2_x,point_2_y,center_x,center_y,distance,reference_theta):
        # 3 점 사이 각
        theta = atan((point_2_y - center_y)/(point_2_x - center_x)) - atan((point_1_y - center_y)/(point_1_x - center_x))
        # 반복 횟수
        repeat = distance * abs(theta) / self._tick_distance

        x_ = []
        y_ = []

        for i in range(int(repeat + 1)):
            if point_1_x < center_x:
                x_.append(center_x - distance*cos(reference_theta + (theta + (self._tick_distance/distance)*i)))
                y_.append(center_y - distance*sin(reference_theta + (theta + (self._tick_distance/distance)*i)))
            else:
                x_.append(center_x + distance*cos(reference_theta + (theta + (self._tick_distance/distance)*i)))
                y_.append(center_y + distance*sin(reference_theta + (theta + (self._tick_distance/distance)*i)))

        temp_theta = atan2(theta,distance)

        return x_,y_

    def calc_straight_distance(self, pos1_x, pos1_y, pos2_x, pos2_y):
        """ 직선 거리 계산 """
        dx = pos1_x - pos2_x
        dy = pos1_y - pos2_y

        return sqrt(dx * dx + dy * dy)    

    def set_vehicle_pose(self, status_msg):
        self.vehicle_yaw = status_msg.heading#yaw
        self.vehicle_rear_position =  Point(status_msg.position.x,status_msg.position.y,status_msg.position.z)
        self.vehicle_front_position = Point(
            self.vehicle_rear_position.x + self.wheelbase * cos(radians(self.vehicle_yaw)),
            self.vehicle_rear_position.y + self.wheelbase * sin(radians(self.vehicle_yaw)),
            self.vehicle_rear_position.z
        )

    def change_status(self):
        if self.parking_status == ParallelParkingStatus.searching:
            self.next_parking_status = ParallelParkingStatus.IN_Phase_1
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.IN_Phase_1:
            self.next_parking_status = ParallelParkingStatus.IN_Phase_2
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.IN_Phase_2:
            self.next_parking_status = ParallelParkingStatus.IN_Phase_3
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.IN_Phase_3:
            if self.is_parallel_to_parking_space():
                self.next_parking_status = ParallelParkingStatus.parking
            else:
                self.next_parking_status = ParallelParkingStatus.IN_Phase_4
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.IN_Phase_4:
            if self.is_parallel_to_parking_space():
                self.next_parking_status = ParallelParkingStatus.parking
            else:
                self.next_parking_status = ParallelParkingStatus.IN_Phase_3
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.parking:
            self.next_parking_status = ParallelParkingStatus.Idle
            self.parking_status = ParallelParkingStatus.waiting

        elif self.parking_status == ParallelParkingStatus.waiting:
            self.parking_status = self.next_parking_status

    def is_parallel_to_parking_space(self) :
        vehicle_yaw = self.vehicle_yaw
        if vehicle_yaw < 0:
            vehicle_yaw = 180 - vehicle_yaw
        parking_space_heading = degrees(self.parallel_heading)
        if parking_space_heading < 0:
            parking_space_heading = 180 - parking_space_heading
        return abs(parking_space_heading - vehicle_yaw) < 5

    def is_phase_end(self, goal_point):
        def _is_distance_to_goal_point(_in, position):
            dist = sqrt(pow(goal_point.x-position.x,2) + pow(goal_point.y-position.y, 2))            
            return dist < _in

        if self.parking_status == ParallelParkingStatus.IN_Phase_1:
            return _is_distance_to_goal_point(_in=0.8, position=self.vehicle_rear_position)
        elif self.parking_status == ParallelParkingStatus.IN_Phase_2:
            return _is_distance_to_goal_point(_in=self.rear_overhang * 1.2, position=self.vehicle_rear_position)
        elif self.parking_status == ParallelParkingStatus.IN_Phase_3:
            return _is_distance_to_goal_point(_in=self.front_overhang * 1.2, position=self.vehicle_front_position)
        elif self.parking_status == ParallelParkingStatus.IN_Phase_4:
            return _is_distance_to_goal_point(_in=self.rear_overhang * 1.2, position=self.vehicle_rear_position)

    def IN_Phase_1(self, link_path):
        print("IN_Phase_1")

        #find cureent_waypoint
        min = float('inf')
        for index, points in enumerate(link_path.poses):
            dx = points.pose.position.x - self.vehicle_rear_position.x 
            dy = points.pose.position.y - self.vehicle_rear_position.y
            dist = sqrt(pow(dx,2)+pow(dy,2))
            if dist < min :
                min = dist
                current_waypoint = index
        
        positions = Path()
        positions.header.frame_id='/map'
        for points in link_path.poses[0:current_waypoint]:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = points.pose.position.x
            tmp_pose.pose.position.y = points.pose.position.y
            tmp_pose.pose.position.z = points.pose.position.z            
            tmp_pose.pose.orientation.x = points.pose.orientation.x
            tmp_pose.pose.orientation.y = points.pose.orientation.y
            tmp_pose.pose.orientation.z = points.pose.orientation.z
            tmp_pose.pose.orientation.w = points.pose.orientation.w
            positions.poses.append(tmp_pose)

        self.searching_start_point = Point(self.parking_start_point.x - 45*cos(self.parallel_heading), self.parking_start_point.y - 45*sin(self.parallel_heading), 0.)
        self.searching_end_point = Point(self.parking_start_point.x + 45*cos(self.parallel_heading), self.parking_start_point.y + 45*sin(self.parallel_heading), 0.)        
        
        path_distance = self.calc_straight_distance(self.searching_start_point.x,self.searching_start_point.y,self.searching_end_point.x,self.searching_end_point.y)
        searching_path_repeat = path_distance/self._tick_distance         
        theta=atan2(self.searching_end_point.y-self.searching_start_point.y,self.searching_end_point.x-self.searching_start_point.x)
        ratation_matric_1 = np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])
        
        for k in range(0,int(searching_path_repeat+1)):
            ratation_matric_2 = np.array([[k*self._tick_distance],[0]])
            roation_matric_calc = np.matmul(ratation_matric_1,ratation_matric_2)
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = self.searching_start_point.x + roation_matric_calc[0][0]
            tmp_pose.pose.position.y = self.searching_start_point.y + roation_matric_calc[1][0]
            tmp_pose.pose.position.z = 0
            positions.poses.append(tmp_pose)

        path = positions        
        return path, self.parking_start_point

    def IN_Phase_2(self,):
        print("IN_Phase_2")
        min_r = self.wheelbase / np.tan(self.max_steer * pi/180)  # Re

        C_1_x,C_1_y = self.rotation_matrix_inv(self.parallel_heading,self.goal_point.x,self.goal_point.y,0,min_r) 
        self.C_1 = Point(C_1_x, C_1_y, 0.) #Point_C1

        #차량과 주차공간이 평행하다는 가정하에 진행 (Reinitr == Relmin) 
        temp_x,temp_y = self.rotation_matrix_inv(self.parallel_heading,self.parking_start_point.x,self.parking_start_point.y,0,-min_r)

        C_2_theta = atan2((temp_y - self.goal_point.y),(temp_x - self.goal_point.x)) - atan2((C_1_y - self.goal_point.y),(C_1_x - self.goal_point.x)) #a
        temp_dis = self.calc_straight_distance(self.parking_start_point.x,self.parking_start_point.y,C_1_x,C_1_y) #Dcieinit

        R_c_2 = (pow(temp_dis,2)- pow(min_r,2))/(2*min_r+2*temp_dis*cos(C_2_theta))#Reinitr

        ratation_matric_1 = np.array([[cos((-90*pi/180)+self.parallel_heading),-sin((-90*pi/180)+self.parallel_heading)],
                                      [sin((-90*pi/180)+self.parallel_heading),cos((-90*pi/180)+self.parallel_heading)]])
        ratation_matric_2 = np.array([[R_c_2],[0]])
        roation_matric_calc = np.matmul(ratation_matric_1,ratation_matric_2)
        
        #offset 
        C_2_x = self.parking_start_point.x+roation_matric_calc[0][0]
        C_2_y = self.parking_start_point.y+roation_matric_calc[1][0]

        self.C_2 = Point(C_2_x, C_2_y, 0.) #Point Cr

        # 두 원 사이 점
        b_theta = atan2(self.C_1.y - self.C_2.y,self.C_1.x - self.C_2.x)

        b_x = C_2_x + R_c_2*cos(b_theta)
        b_y = C_2_y + R_c_2*sin(b_theta)

        x_2 , y_2 = self.circle_arc_point(b_x,b_y,self.parking_start_point.x,self.parking_start_point.y,C_2_x,C_2_y,R_c_2,b_theta)
        x_1 , y_1 = self.circle_arc_point(b_x,b_y,self.goal_point.x,self.goal_point.y,C_1_x,C_1_y,min_r,b_theta)

        x_1.reverse()
        y_1.reverse()

        x__ = x_2 + x_1
        y__ = y_2 + y_1

        path = Path()
        path.header.frame_id='/map'
        for i in range(0, len(x__)):            
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = x__[i]
            tmp_pose.pose.position.y = y__[i]
            tmp_pose.pose.position.z = 0
            path.poses.append(tmp_pose)

        return path, self.end_point_1

    def IN_Phase_3(self,):
        print("IN_Phase_3")
        if self.parallel_heading*180/pi - self.vehicle_yaw < 0: # 차량이 왼쪽을 바라보고 있는 
            x_start,x_end,y_start,y_end,t = self.changecoordinate_for_lc(self.end_point_1, self.point_D,self.goal_point)            
        elif self.parallel_heading*180/pi - self.vehicle_yaw > 0: # 차량이 오른쪽을 바라보고 있는 상태
            x_start,x_end,y_start,y_end,t = self.changecoordinate_for_lc(self.end_point_1, self.point_B,self.goal_point)
            
        waypoints_x=[]
        waypoints_y=[]

        x_interval=0.1
        x_num=x_end/x_interval

        for i in range(int(x_start),int(x_num)) : 
            waypoints_x.append(i*x_interval)

        ''' 
        f(x) = -a * sin(b*x + c) + d
        '''

        a = -y_end/2
        b = 0.31/(x_end/9.446)
        c = pi/2
        d = y_end/2
        
        for i in waypoints_x :
            result=a*np.sin(b*i + c) + d
            waypoints_y.append(result)
        

        path = Path()
        path.header.frame_id='/map'
        for i in range(0, len(waypoints_y)):
            local_result=np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
            global_result=t.dot(local_result)
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = global_result[0][0]
            tmp_pose.pose.position.y = global_result[1][0]
            tmp_pose.pose.position.z = 0
            path.poses.append(tmp_pose)

        return path, self.end_point_2

    def IN_Phase_4(self,):
        print("IN_Phase_4")
        if self.parallel_heading*180/pi - self.vehicle_yaw < 0: # 차량이 왼쪽을 바라보고 있는 상태
            x_start,x_end,y_start,y_end,t = self.changecoordinate_for_lc(self.end_point_2, self.point_A,self.goal_point)
        elif self.parallel_heading*180/pi - self.vehicle_yaw > 0: # 차량이 오른쪽을 바라보고 있는 상태
            x_start,x_end,y_start,y_end,t = self.changecoordinate_for_lc(self.end_point_2, self.point_C,self.goal_point)

        waypoints_x=[]
        waypoints_y=[]

        x_interval=0.1
        x_num=x_end/x_interval
        for i in range(int(x_start),int(x_num)) : 
            waypoints_x.append(i*x_interval)

        ''' 
        f(x) = -a * sin(b*x + c) + d
        '''

        a = -y_end/2
        b = 0.31/(x_end/9.446)
        c = pi/2
        d = y_end/2

        for i in waypoints_x :
            result=a*np.sin(b*i + c) + d
            waypoints_y.append(result)

        path = Path()
        path.header.frame_id='/map'
        for i in range(0,len(waypoints_y)) :
            local_result=np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
            global_result=t.dot(local_result)
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = global_result[0][0]
            tmp_pose.pose.position.y = global_result[1][0]
            tmp_pose.pose.position.z = 0
            path.poses.append(tmp_pose)

        return path, self.end_point_1
      

    def waiting_gear(self):
        if self.next_parking_status == ParallelParkingStatus.IN_Phase_1:
            return GearChange.DRIVE
        elif self.next_parking_status == ParallelParkingStatus.IN_Phase_2:
            return GearChange.REVERSE
        elif self.next_parking_status == ParallelParkingStatus.IN_Phase_3:
            return GearChange.DRIVE
        elif self.next_parking_status == ParallelParkingStatus.IN_Phase_4:
            return GearChange.REVERSE        
        elif self.next_parking_status == ParallelParkingStatus.parking:
            return GearChange.PARK
        elif self.next_parking_status == ParallelParkingStatus.Idle:
            return GearChange.DRIVE
        else:
            return GearChange.NONE


class planner :

    def __init__(self):
        rospy.init_node('parking_planner', anonymous=True)

        #publihser
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        path_pub = rospy.Publisher('/parking_path',Path, queue_size=1)

        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB)

        #service        
        rospy.wait_for_service('/Service_MoraiEventCmd')
        event_mode_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv, MoraiEventCmdSrv)

        #def
        stanley_ = Stanley_alg(1.0,0.7, 2.7, 36)
        self.parallel = parallel_parking()
        self.status_msg = EgoVehicleStatus()
        path_reader = pathReader('chapter7')       

        self.ctrl_msg = CtrlCmd()
        self.ctrl_msg.longlCmdType = 2

        self.morai_event = EventInfo()                
        self.morai_event.option = 3
        self.morai_event.ctrl_mode = 3
   
        self.during_time = 0
        self.current_time = 0
        self.time_step = 0.333
        self.target_velocity = 0
        
        self.is_status = False
        self.is_complete = False
        
        link_path = path_reader.read_txt("A219BS010377.txt")

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            
            if self.is_status:
                self.parallel.set_vehicle_pose(self.status_msg)
                                
                if self.parallel.parking_status == ParallelParkingStatus.Idle:                    
                    self.global_path, self.goal_point = self.parallel.IN_Phase_1(link_path)     
                    self.gear = GearChange.DRIVE
                    self.parallel.parking_status = ParallelParkingStatus.IN_Phase_1   

                elif self.parallel.parking_status == ParallelParkingStatus.IN_Phase_1:
                    self.target_velocity = 10
                    if self.parallel.is_phase_end(self.goal_point):
                        self.target_velocity = 0
                        self.parallel.change_status()
                        self.global_path, self.goal_point = self.parallel.IN_Phase_2()
                
                elif self.parallel.parking_status == ParallelParkingStatus.IN_Phase_2:
                    self.target_velocity = 3
                    
                    if self.parallel.is_phase_end(self.goal_point):
                        self.parallel.change_status()
                        self.global_path, self.goal_point = self.parallel.IN_Phase_3()

                elif self.parallel.parking_status == ParallelParkingStatus.IN_Phase_3:
                    self.target_velocity = 3
                    if self.parallel.is_phase_end(self.goal_point):
                        self.target_velocity = 0
                        self.parallel.change_status()
                        if not self.parallel.is_parallel_to_parking_space():
                            self.global_path, self.goal_point = self.parallel.IN_Phase_4()

                elif self.parallel.parking_status == ParallelParkingStatus.IN_Phase_4:
                    self.target_velocity = 3
                    if self.parallel.is_phase_end(self.goal_point):
                        self.target_velocity = 0
                        if not self.parallel.is_parallel_to_parking_space():
                            self.global_path, self.goal_point = self.parallel.IN_Phase_3()
                        else:
                            self.is_complete = True               
                            self.target_velocity = 0           
                            self.ctrl_msg.steering = 0
                            self.gear = GearChange.PARK
                            print("done")

                elif self.parallel.parking_status == ParallelParkingStatus.waiting:
                    self.target_velocity = 0
                    self.during_time += self.time_step
                    if self.during_time > 3:
                        self.during_time = 0
                        if abs(self.status_msg.velocity.x) < 0.05:
                            self.gear = self.parallel.waiting_gear()
                            self.parallel.change_status()


                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint = findLocalPath(self.global_path,self.status_msg)

                stanley_.get_path(local_path)
                stanley_.get_vehicle_state(self.status_msg)                               
                self.morai_event.gear = self.gear
                status_result = event_mode_srv(self.morai_event)                
                if not self.is_complete:
                    self.ctrl_msg.steering = stanley_.calculate_steering_angle(status_result.response.gear)

                self.ctrl_msg.velocity = self.target_velocity
                
                ctrl_pub.publish(self.ctrl_msg) ## Vehicle Control
                path_pub.publish(self.global_path)
                if self.is_complete:
                    break
                rate.sleep()
    
    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg = data
        self.is_status = True


if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass