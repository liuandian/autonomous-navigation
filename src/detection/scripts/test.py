#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import cv2
import easyocr
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from sklearn.decomposition import PCA
import time
import math
from collections import defaultdict
from std_msgs.msg import Int8
# 关键：引入 message_filters
import message_filters

import threading

class BoxCountNode:
    def __init__(self):
        #rospy.init_node("ocr_lidar_box_center_node")
        rospy.init_node("ocr_lidar_box_center_node", disable_signals=True)


        # 订阅相机内参（通常可以不用时间同步）
       
        rospy.loginfo(f"test!!!!!!!!!!!")
        # image_sub = message_filters.Subscriber("/front/image_raw", Image)
        # lidar_sub = message_filters.Subscriber("/mid/points", PointCloud2)

        rospy.Subscriber("/mid/points", PointCloud2, self.lidar_callback, queue_size=1)
        rospy.Subscriber("/front/image_raw", Image, self.image_callback, queue_size=10)
        # 创建近似时间同步器
        # sync = message_filters.ApproximateTimeSynchronizer(
        #     [image_sub, lidar_sub],
        #     queue_size=50,  # 队列大小
        #     slop=0.2       # 允许的时间差（单位：秒）
        # )
        # sync.registerCallback(self.synced_callback)
        self.start_time = time.time()
        self.start_time_2 = time.time()
        self.start_time_3 = time.time()
        self.end_time = time.time()
        self.end_time_2 = time.time()
        self.end_time_3 = time.time()
    def lidar_callback(self, pc2_msg):
        self.end_time_2 = time.time()
        rospy.loginfo(f"LIdar  Time elapsed: {self.end_time_2 - self.start_time_2:.2f} seconds")
        self.start_time_2 = self.end_time_2
        return
        self.lidar_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)

    def image_callback(self, img_msg):
        self.end_time_3 = time.time()
        #rospy.loginfo(f"Camera  Time elapsed: {self.end_time_3 - self.start_time_3:.2f} seconds")
        self.start_time_3 = self.end_time_3
        
        return
    def synced_callback(self, img_msg, pc2_msg):
        self.end_time = time.time()
        diff = abs((img_msg.header.stamp - pc2_msg.header.stamp).to_sec())
        #rospy.loginfo(f"Fuse ts Δt: {diff:.3f}, Time elapsed: {self.end_time - self.start_time:.2f} seconds")
        self.start_time = self.end_time
        return

if __name__ == "__main__":
    node = BoxCountNode()
    rospy.spin()

