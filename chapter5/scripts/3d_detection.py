#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

# 포인트 클라우드 콜백 함수
def point_cloud_callback(msg):
    # 포인트 클라우드 데이터를 numpy 배열로 변환
    cloud_points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)), dtype=np.float32)
    
    # 포인트 클라우드를 Open3D 형식으로 변환
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_points[:, :3])
    
    # 간단한 바운딩 박스 생성 (여기서는 최소-최대 좌표 기준)
    aabb = pcd.get_axis_aligned_bounding_box()
    
    # 바운딩 박스 생성
    create_3d_bounding_boxes(aabb, msg.header.frame_id)

def create_3d_bounding_boxes(aabb, frame_id):
    marker_array = MarkerArray()
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = 0

    # 바운딩 박스의 크기 설정
    extent = aabb.get_extent()
    marker.scale.x = extent[0]
    marker.scale.y = extent[1]
    marker.scale.z = extent[2]
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # 바운딩 박스의 중심 좌표 설정
    center = aabb.get_center()
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation.w = 1.0  # 필요에 따라 적절한 회전값으로 설정

    marker_array.markers.append(marker)
    
    marker_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('open3d_lidar_node')
    rospy.Subscriber('/lidar_points', PointCloud2, point_cloud_callback)
    marker_pub = rospy.Publisher('/bounding_box_markers', MarkerArray, queue_size=10)
    rospy.spin()
