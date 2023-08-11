#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from asyncore import read
import sys,os, time
from this import d

# from click import open_file

import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd,GetTrafficLightStatus,SetTrafficLight, SyncModeCmd, SyncModeCmdResponse, WaitForTick, WaitForTickResponse, EventInfo, SyncModeCtrlCmd, MultiEgoSetting, SyncModeScenarioLoad, ScenarioLoad, SyncModeSetGear 
from morai_msgs.srv import MoraiSyncModeCmdSrv ,MoraiWaitForTickSrv , MoraiEventCmdSrv ,MoraiScenarioLoadSrvRequest , MoraiSyncModeCtrlCmdSrv, MoraiSyncModeSLSrv, MoraiScenarioLoadSrv, MoraiSyncModeSetGearSrv
from lib.class_defs import mgeo_planner_map
from lib.class_defs import *
from lib.utils import pathReader, findLocalPath,purePursuit,pidController,velocityPlanning

from model.ActorNet import *
from model.AgentNet import *
from model.CriticNet import *
from model.Buffer import *

import tf
from math import cos,sin,sqrt,pow,atan2,pi
import gym

TIMES_STEP = 20
class action_space(object):
    def __init__(self, dim, high, low, seed):
        self.shape = (dim,)
        self.high = np.array(high)
        self.low = np.array(low)
        self.seed = seed
        assert(dim == len(high) == len(low))
        np.random.seed(self.seed)

    def sample(self):
        return np.random.uniform(self.low, self.high)

class observation_space(object):
    def __init__(self, dim, high=None, low=None, seed=None):
        self.shape = (dim,)
        self.high = high
        self.low = low
        self.seed = seed


class Morai_DDPG(gym.Env):
    def __init__(self, action_lambda=0.5, SEED=1):
        rospy.init_node("morai_env", anonymous=True)
        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        sensor_capture_mode=bool(arg[2])
        rospy.Rate(30)
        self.time_step = 0
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('chapter6')
        
        load_path = pkg_path + '/scripts/R_KR_PG_K-City'
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.temp_link = Link()
        self.temp_node = Node()
        
        self.reset_v = None
        self.stop_time = 0
        
        self.action_lambda = action_lambda
        self.prev_action = (0.0, 0.5)       
        
        #service 설정
        rospy.wait_for_service('/SyncModeCmd')
        rospy.wait_for_service('/SyncModeWaitForTick')
        rospy.wait_for_service('/SyncModeCtrlCmd')
        rospy.wait_for_service('/Service_MoraiSL')
        rospy.wait_for_service('/SyncModeSetGear')
        
        scenario_setting = ScenarioLoad()
        scenario_setting.file_name                      = "morai_scen"
        scenario_setting.load_network_connection_data   = False
        scenario_setting.delete_all                     = False
        scenario_setting.load_ego_vehicle_data          = True
        scenario_setting.load_surrounding_vehicle_data  = True
        scenario_setting.load_pedestrian_data           = True
        scenario_setting.load_obstacle_data             = True
        scenario_setting.set_pause                      = False
        self.SL = scenario_setting
        
        self.ros_SL_srv = rospy.ServiceProxy('/Service_MoraiSL', MoraiScenarioLoadSrv)
        result = self.ros_SL_srv(self.SL)
        self.sync_mode_srv = rospy.ServiceProxy('SyncModeCmd', MoraiSyncModeCmdSrv)
        self.tick_wait_srv = rospy.ServiceProxy('SyncModeWaitForTick', MoraiWaitForTickSrv)
        self.ctrl_cmd_srv = rospy.ServiceProxy('SyncModeCtrlCmd', MoraiSyncModeCtrlCmdSrv)
        self.gear_set_srv = rospy.ServiceProxy('SyncModeSetGear', MoraiSyncModeSetGearSrv)
        
        # 싱크로노스 모드 시작 설정
        self.sync_mode_on = SyncModeCmd() 
        self.sync_mode_on.user_id = "sync_master"
        self.sync_mode_on.time_step = 20 # 20ms # 20의 배수로 가능 한데 40이면 한번 보낼때 2프레임씩  
        self.sync_mode_on.start_sync_mode = True
        TIMES_STEP = 20
        self.frame_step = self.sync_mode_on.time_step / TIMES_STEP 
        
        self.sync_mode_resp = self.sync_mode_srv(self.sync_mode_on)      
        
        
        self.sync_set_gear = SyncModeSetGear()
        self.sync_set_gear.gear = 4
        self.sync_set_gear.frame = self.sync_mode_resp.response.frame
        res = self.gear_set_srv(self.sync_set_gear)
        # print(res)
        self.next_frame = self.sync_mode_resp.response.frame
  
        self.tick=WaitForTick()
        self.tick.user_id = self.sync_mode_resp.response.user_id
        self.tick.frame = self.next_frame
        tick_resp = self.tick_wait_srv(self.tick)         
        result = self.ros_SL_srv(self.SL)    
        
        self.ego_status = tick_resp.response.vehicle_status
        self.ctrl_cmd_msg = SyncModeCtrlCmd()
        self.ctrl_cmd_msg.sensor_capture = False        
        self.action_generator = action_space(2, (0.6326, 1.0), (-0.6326, -1.0), SEED)
        self.rate = rospy.Rate(30)
        self.prev_time = time.time()
        self.check_v_cnt = 0
        self.reward = 0
        self.done = False
        self.prev_obs = [] 
        self.obs = self.ego_status
        
    def get_link(self,idx):
        return self.links[idx]
        
    def _step(self, act):
        steer = act[0]
        
        if act[1] >= 0:
            acc = act[1]
            brake = 0
        else:
            brake = abs(act[1])
            acc = 0
        
        self.steer = steer * self.action_lambda + (1 - self.action_lambda) * self.prev_action[0]
       
            
        if self.prev_action[1] >= 0:
            self.acc = acc * self.action_lambda + (1 - self.action_lambda) * self.prev_action[1]
            self.brake = brake * self.action_lambda
        else:
            self.brake = brake * self.action_lambda + (1 - self.action_lambda) * self.prev_action[1]
            self.acc = acc * self.action_lambda

        self.ctrl_cmd([self.acc, self.brake, self.steer])        
        self.prev_action = act
        
        self.obs = self.make_obs()
        if self.prev_obs == []:
            self.prev_obs = self.obs
       
        self.reward = self._get_reward(self.obs, self.prev_obs)
        self.rate.sleep()
        return self.obs, self.reward, self.episode_end, self.reset_v   
    
    def make_obs(self):
        ego_stat = self.ego_status
        self.ego_yaw = (self.ego_status.heading / 180) * np.pi
        if self.prev_obs != []:
            self.prev_obs = self.obs        
        rotate_p = Point()
        self.is_look_forward_p = False
        
        min_dist = None
        link_name = ["A219BS010394", "A219BS010657", "A219BS010396", "A219BS010665", "A219BS010399", "A219BS010675"]
        self.min_dist  = float('inf')
        tmp_points = None 
        for i in link_name:
            tmp_link = self.get_link(i)
            tmp_point = tmp_link.points 
            
            for i in range(len(tmp_point)):
                x = tmp_point[i][0]
                y = tmp_point[i][1]
                dx = ego_stat.position.x - x
                dy = ego_stat.position.y - y            
                dist = dx * dx + dy * dy    
                if dist < self.min_dist:
                    self.min_dist = dist
                    min_idx = i
                    tmp_points = tmp_point
        if min_idx == len(tmp_points) - 1:
            vec1 = (tmp_points[min_idx][0] - ego_stat.position.x,tmp_points[min_idx][1] - ego_stat.position.y)
            vec2 = (tmp_points[min_idx - 1][0] - ego_stat.position.x,tmp_points[min_idx - 1][1] - ego_stat.position.y)
            m1 = (tmp_points[min_idx - 1][1] - tmp_points[min_idx][1]) / (tmp_points[min_idx - 1][0] - tmp_points[min_idx][0])
        else:
            vec1 = (tmp_points[min_idx][0] - ego_stat.position.x,tmp_points[min_idx][1] - ego_stat.position.y)
            vec2 = (tmp_points[min_idx + 1][0] - ego_stat.position.x,tmp_points[min_idx + 1][1] - ego_stat.position.y)
            m1 = (tmp_points[min_idx+1][1] - tmp_points[min_idx][1]) / (tmp_points[min_idx+1][0] - tmp_points[min_idx][0])
        
        if min_idx == len(tmp_points) - 1:
            vec_road = (tmp_points[min_idx][0] - tmp_points[min_idx - 1][0],tmp_points[min_idx][1] - tmp_points[min_idx - 1][1])
        else:
            vec_road = (tmp_points[min_idx][0] - tmp_points[min_idx + 1][0],tmp_points[min_idx][1] - tmp_points[min_idx + 1][1])
        
        vec_car = (ego_stat.velocity.x, ego_stat.velocity.y)   
        cos_theta = (vec_car[0] * vec_road[0] + vec_car[1] * vec_road[1]) / (sqrt(vec_road[0] * vec_road[0] + vec_road[1] * vec_road[1])*sqrt(vec_car[0] * vec_car[0] + vec_car[1] * vec_car[1]))
        self.v_road = abs(sqrt(vec_car[0] * vec_car[0] + vec_car[1] * vec_car[1]) * cos_theta)
        
        b1 = -m1 * tmp_points[min_idx][0] + tmp_points[min_idx][1]
        m2 = -1.0 / m1
        b2 = -m2 * ego_stat.position.x + ego_stat.position.y
        mid_x = (b2 - b1)/(m1-m2)
        mid_y = m1 * (-(b1 - b2) / (m1 - m2)) + b1
        
       
        ego_car = {}
        ego_car["speedX"] = ego_stat.velocity.x
        ego_car["speedY"] = ego_stat.velocity.y
        ego_car["angle"] = (ego_stat.wheel_angle * np.pi / 180) / np.pi
        ego_car["damage"] = 0        

        
        if vec1[0] * vec2[0] < 0 or vec1[1] * vec2[1] < 0:
            min_idx += 1
        
        mid_x, mid_y = self.make_car_coordinate(ego_stat.position.x, ego_stat.position.y, mid_x, mid_y,self.ego_yaw)
        
        lane_width = 3.6
        
        abs_dy = abs(mid_y)
        
        if mid_y >= 0:
            ego_car["mid_road"] = (abs_dy /(lane_width / 2)) 
        elif mid_y < 0:
            ego_car["mid_road"] = -(abs_dy /(lane_width / 2)) 
            
        car_wp_x, car_wp_y = self.make_car_coordinate(ego_stat.position.x, ego_stat.position.y, 
                                                      tmp_points[min_idx][0], tmp_points[min_idx][1],self.ego_yaw)
        
        
        ego_car["waypoint_x"] = car_wp_x
        ego_car["waypoint_y"] = car_wp_y
        
        self.mid_reward = np.exp(abs_dy) - 1
        self.speed = sqrt(ego_stat.velocity.x * ego_stat.velocity.x + ego_stat.velocity.y * ego_stat.velocity.y)
        obs = list(ego_car.items())
        ob = []
        for j in obs:
            ob.append(j[1])
        obs = np.array(ob)
        self.prev_speed = self.speed
        return list(obs) + list(self.prev_action)
    
    def make_car_coordinate(self, car_x, car_y, map_x, map_y, car_heading):
        x = float(map_x) - car_x 
        y = float(map_y) - car_y
        
        rotate_x = x * cos(car_heading) + y * sin(car_heading)
        rotate_y = -x * sin(car_heading) + y * cos(car_heading)
        
        return rotate_x, rotate_y
        
        
        
    def _reset(self,SEED=1):
        self.time_step = 0
        self.stop_time = 0 
        self.episode_end = False
        self.reset_v = False
        self.sync_mode_on.start_sync_mode = False

        # tick_resp = self.tick_wait_srv(self.tick)     
        self.sync_mode_resp = self.sync_mode_srv(self.sync_mode_on)
        
        result = self.ros_SL_srv(self.SL)
        
        self.sync_mode_on.start_sync_mode = True
        self.sync_mode_resp = self.sync_mode_srv(self.sync_mode_on)
        self.next_frame = self.sync_mode_resp.response.frame
        
        self.sync_set_gear.frame = self.sync_mode_resp.response.frame
        res = self.gear_set_srv(self.sync_set_gear)
        self.tick=WaitForTick()
        self.tick.user_id = self.sync_mode_resp.response.user_id
        self.tick.frame = self.next_frame
        tick_resp = self.tick_wait_srv(self.tick)
        self.ego_status = tick_resp.response.vehicle_status
        
        
        
        
        self.prev_action = self.action_generator.sample()
        self.prev_obs = []
        self.prev_speed = sqrt(self.ego_status.velocity.x * self.ego_status.velocity.x + 
                               self.ego_status.velocity.y * self.ego_status.velocity.y)
        
        print(self.prev_action)

        self.check_v_cnt = 0


        return self.make_obs()
    
    def _get_reward(self, obs, prev_obs):
        reward = 0        
        self.cur_ego_x = obs[5]
        self.cur_ego_y = obs[6]
        angle = obs[2]
        m_r = obs[4]
        speed = sqrt(self.ego_status.velocity.x * self.ego_status.velocity.x + 
                     self.ego_status.velocity.y * self.ego_status.velocity.y)
        cur_damage = obs[3]
        prev_damage = prev_obs[3]
        
        if cur_damage - prev_damage > 0:
            reward = -1

        if self.mid_reward >= 0 and  self.mid_reward <= 1:
            reward = (speed * np.cos(angle * np.pi / 180) - 
                      np.abs(speed* np.sin(angle * np.pi / 180)) - speed * (np.abs(m_r)))
        else:
            reward = -2
            self.episode_end = True
            self.reset_v = True
        
        if self.prev_speed < 3 and speed < 3:
            
            self.stop_time += 1
            if self.stop_time > 100:
                reward = -2
                print("this")
                self.episode_end = True
                self.reset_v = True
        else:
            self.stop_time = 0
        self.prev_speed = speed    
        self.time_step += 1
        
        return reward
        
        
    
    #################################################################################
    def ctrl_cmd(self,act):
        # 액션 커맨드 전송(프레임단위)
        self.ctrl_cmd_msg.command.longlCmdType = 1
        self.ctrl_cmd_msg.command.accel = act[0]
        self.ctrl_cmd_msg.command.brake = act[1]
        self.ctrl_cmd_msg.command.steering = act[2]
        # print("value acc, brake:",act[0], act[1])
        tick_resp = self.tick_wait_srv(self.tick) 
        
        self.ctrl_cmd_msg.frame = tick_resp.response.frame 
        ctrl_cmd_resp = self.ctrl_cmd_srv(self.ctrl_cmd_msg)
        
        tick_resp = self.tick_wait_srv(self.tick)
        self.ego_status = tick_resp.response.vehicle_status
        
        
    

if __name__ == '__main__':
    try:
        print("morai before")
        env = Morai_DDPG()
        print("morai after")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('chapter6')
        model_path = pkg_path + '/scripts/model_save'
        agent = Agent(alpha=0.000025, beta=0.00025, input_dims=[9], tau=0.001, env=env,batch_size=512,  layer1_size=400, layer2_size=300, n_actions=2, save_dir=model_path)
        first = True
        # agent.load_models()
        score_history = []

        device = T.device("cuda" if T.cuda.is_available() else "cpu")
        
        for episode in range(100000):
        
            state = env._reset()
            
            episode_reward = 0
            done = False
            re_v = False
            
            while not done:
                action = agent.choose_action(state)
                new_state, reward, done, _ = env._step(action)
                agent.remember(state, action, reward, new_state, int(done))
                agent.learn()
                episode_reward += reward
                state = new_state
                
            agent.save_models()
            print(episode_reward)
            score_history.append(episode_reward)
            
            print('episode ', episode, 'score %.2f' % episode_reward,'trailing 100 games avg %.3f' % np.mean(score_history[-100:]))
                                
        
    except rospy.ROSInterruptException:
        pass
