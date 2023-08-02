#!/usr/bin/env python3
from network.ros_manager_cam import RosManager
import sys

def main():
    print("here")
    ros_manager = RosManager()
    print("execute")
    ros_manager.execute()

if __name__ == '__main__':
    main()