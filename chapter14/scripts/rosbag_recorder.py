#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import rosbag
import time
from morai_msgs.msg import CtrlCmd

class rosbag_recorder :
    def __init__(self):
        rospy.init_node('rosbag_recorder')
        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.ctrlcmd_cb)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('chapter14')
        bag_file_path = pkg_path + '/'+'ctrlcmd.bag'      

        self.bag_wrtier = rosbag.Bag(bag_file_path, 'w')        

        while not rospy.is_shutdown():
            pass

        self.bag_wrtier.close()


    def ctrlcmd_cb(self, cmd):        
        try:
            self.bag_wrtier.write('/ctrl_cmd',cmd)
            rospy.loginfo('ctrl_cmd_writhe')
        except:
            pass


if __name__ == '__main__':
    try:
        rosbag_recorder()
    except rospy.ROSInterruptException:
        pass
