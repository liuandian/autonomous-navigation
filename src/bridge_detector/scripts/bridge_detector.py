#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np

class BridgeDetector:
    def __init__(self):
        # 仅在实例化时初始化节点，否则会报错
        if not rospy.core.is_initialized():
            rospy.init_node('bridge_detector', anonymous=True)

        self.map_topic = rospy.get_param('~map_topic', '/move_base/global_costmap/costmap')
        self.bridge_y_min = rospy.get_param('~bridge_y_min', -22.0)
        self.bridge_y_max = rospy.get_param('~bridge_y_max', -2.0)
        self.occ_thresh = rospy.get_param('~occ_thresh', 50)
        self.x_center_approx = rospy.get_param('~x_center_approx', 7.0)
        self.search_width = rospy.get_param('~search_width', 2.0)

        self.map_msg = None
        self.reached_goal = False

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        self.pose2d_pub = rospy.Publisher('/custom_goal_pose2d', Pose2D, queue_size=5)
        self.marker_pub = rospy.Publisher('/bridge_marker', Marker, queue_size=1)
        self.unlock_pub = rospy.Publisher('/cmd_open_bridge', Bool, queue_size=1)

        rospy.loginfo("🛠 正在准备检测桥...")


    def map_cb(self, msg):
        self.map_msg = msg
        rospy.loginfo("📥 接收到地图 (%d x %d)", msg.info.width, msg.info.height)

    def result_callback(self, msg):
        if msg.status.status == 3:
            rospy.loginfo("✅ 到达一个路径点")
            self.reached_goal = True
        else:
            rospy.logwarn("⚠️ 导航失败，状态码: %d", msg.status.status)

    def detect_bridge(self):
        if self.map_msg is None:
            rospy.logwarn("❌ 没有可用地图，检测中止")
            return

        info = self.map_msg.info
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        data = np.array(self.map_msg.data).reshape((height, width))

        y_start = int((self.bridge_y_min - origin_y) / res)
        y_end = int((self.bridge_y_max - origin_y) / res)
        center_x_idx = int((self.x_center_approx - origin_x) / res)
        delta = int(self.search_width / res)

        bridge_candidates = []

        for y in range(min(y_start, y_end), max(y_start, y_end)):
            left_x = None
            right_x = None
            for dx in range(1, delta):
                lx = center_x_idx - dx
                rx = center_x_idx + dx
                if lx >= 0 and data[y, lx] >= self.occ_thresh and left_x is None:
                    left_x = lx
                if rx < width and data[y, rx] >= self.occ_thresh and right_x is None:
                    right_x = rx
                if left_x is not None and right_x is not None:
                    break

            if left_x and right_x:
                cx = (left_x + right_x) / 2.0
                wx = cx * res + origin_x
                wy = y * res + origin_y
                bridge_candidates.append((wx, wy))

        if not bridge_candidates:
            rospy.logwarn("❌ 没有检测到桥区域")
            return

        bridge_array = np.array(bridge_candidates)
        mean_x = np.mean(bridge_array[:, 0])
        mean_y = np.mean(bridge_array[:, 1])
        rospy.loginfo("✅ 桥中心: x = %.2f, y = %.2f", mean_x, mean_y)

        for idx, x in enumerate([10.0, 4.0]):
            self.reached_goal = False
            pose2d = Pose2D()
            pose2d.x = x
            pose2d.y = mean_y
            pose2d.theta = 0.0
            self.pose2d_pub.publish(pose2d)
            rospy.loginfo("➡️ 发布目标点 x=%.2f, y=%.2f", x, mean_y)

            timeout = rospy.Time.now() + rospy.Duration(30)
            rate = rospy.Rate(5)
            while not self.reached_goal and rospy.Time.now() < timeout:
                rate.sleep()

            if not self.reached_goal:
                rospy.logwarn("⏰ 超时未到达目标点，停止后续动作")
                break

            if idx == 0:
                rospy.loginfo("🔓 解锁桥梁")
                self.unlock_pub.publish(Bool(data=True))

        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bridge_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = mean_x
        marker.pose.position.y = mean_y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        rospy.loginfo("📤 已发布桥中心 marker")

class BridgeTriggerListener:
    def __init__(self):
        rospy.init_node('bridge_trigger_listener')
        self.trigger_sub = rospy.Subscriber('/bridge_detection_trigger', Bool, self.trigger_cb)
        rospy.loginfo("📡 持续监听桥梁检测触发器...")

    def trigger_cb(self, msg):
        if msg.data:
            rospy.loginfo("📩 收到触发信号，实例化桥梁检测器")
            detector = BridgeDetector()
            detector.detect_bridge()

if __name__ == '__main__':
    try:
        BridgeTriggerListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
