#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import rosbag
import time

from morai_msgs.msg import EventInfo
from morai_msgs.srv import MoraiEventCmdSrv

class rosbag_replay :
    def __init__(self):
        rospy.init_node('rosbag_replay')

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)        
        self.event = EventInfo()

        self.event.option = 1
        self.event.ctrl_mode = 3
        self.event_srv(self.event)

        self.event.option = 2
        self.event.gear = 4
        self.event_srv(self.event)    

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('chapter14')
        bag_file_path = pkg_path + '/'+'ctrlcmd.bag'      
        
        prev_time = None
        rate = 0.0 

        with rosbag.Bag(bag_file_path, 'r') as bag:
            for topic, msg, t in bag.read_messages():                
                if not prev_time :
                    prev_time = t.to_sec()
                else:                    
                    rate = t.to_sec() - prev_time                    
                    prev_time = t.to_sec()
                
                pub = rospy.Publisher(topic, type(msg), queue_size=10)
                pub.publish(msg)
                print('ctrl_cmd_pub')
                time.sleep(rate)

if __name__ == '__main__':
    try:
        rosbag_replay()
    except rospy.ROSInterruptException:
        pass
