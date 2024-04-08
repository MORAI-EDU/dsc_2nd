#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt
from morai_msgs.msg import CtrlCmd

class DataPlotNode:
    def __init__(self):
        rospy.init_node('data_plot_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz로 루프 실행
        self.time_data = []
        self.sensor_data = []

        # 구독자 생성
        rospy.Subscriber('/ctrl_cmd_0', CtrlCmd, self.ctrl_cmd_cb)

        while not rospy.is_shutdown():
            # 데이터 플롯
            plt.clf()
            plt.plot(self.time_data, self.sensor_data)
            plt.xlabel('Time [s]')
            plt.ylabel('Wheel angle[deg]')
            plt.title('Wheel angle Plot')
            plt.pause(0.001)
            # self.rate.sleep()

    def ctrl_cmd_cb(self, msg):                
        self.time_data.append(rospy.get_time())        
        self.sensor_data.append(msg.steering * 13.9 * 180 / 3.14)

    


if __name__ == '__main__':
    try:
        node = DataPlotNode()
        node.plot_data()
    except rospy.ROSInterruptException:
        pass