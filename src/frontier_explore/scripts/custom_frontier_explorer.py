#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
import actionlib
import math

class FrontierExplorer:
    def __init__(self):
        rospy.init_node('custom_frontier_explorer')
        
        # 创建move_base客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")
        
        # 订阅前沿点
        self.frontiers_sub = rospy.Subscriber(
            '/explore/frontiers', MarkerArray, self.frontiers_callback)
        
        self.tf_listener = tf.TransformListener()
        self.is_navigating = False
        self.current_goal = None
        
    def frontiers_callback(self, frontiers_msg):
        if self.is_navigating:
            return  # 如果正在导航，不处理新的前沿点
            
        # 获取机器人当前位置
        try:
            (trans, _) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("获取机器人位置失败，使用(0,0)")
            robot_x, robot_y = 0.0, 0.0
            
        best_frontier = None
        min_distance = float('inf')
        
        # 寻找最近的前沿点
        for marker in frontiers_msg.markers:
            # 只处理POINTS类型的标记
            if marker.type == marker.POINTS and len(marker.points) > 0:
                for point in marker.points:
                    # 计算距离
                    distance = math.sqrt((point.x - robot_x)**2 + (point.y - robot_y)**2)
                    if distance < min_distance and distance > 1.0:  # 忽略太近的点
                        min_distance = distance
                        best_frontier = point
        
        if best_frontier:
            self.navigate_to_frontier(best_frontier)
        else:
            rospy.loginfo("没有找到合适的前沿点")
            
    def navigate_to_frontier(self, frontier_point):
        self.is_navigating = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = frontier_point
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"导航到前沿点: x={frontier_point.x}, y={frontier_point.y}")
        self.current_goal = frontier_point
        
        # 发送导航目标
        self.move_base_client.send_goal(goal, done_cb=self.navigation_done_callback)
        
    def navigation_done_callback(self, status, result):
        self.is_navigating = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("成功到达前沿点")
        else:
            rospy.logwarn(f"导航失败，状态: {status}")
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()