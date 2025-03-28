#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import MarkerArray, Marker
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point

class RegionLimitedExplorer:
    def __init__(self):
        rospy.init_node('region_limited_explorer')
        
        # 定义感兴趣区域范围 [x_min:x_max, y_min:y_max]
        self.x_min = rospy.get_param('~x_min', 20)
        self.x_max = rospy.get_param('~x_max', 30)
        self.y_min = rospy.get_param('~y_min', 20)
        self.y_max = rospy.get_param('~y_max', 30)
        
        # 订阅前沿点
        self.frontiers_sub = rospy.Subscriber('/explore/frontiers', MarkerArray, self.frontiers_callback)
        
        # 发布过滤后的前沿点
        self.filtered_frontiers_pub = rospy.Publisher('/filtered_frontiers', MarkerArray, queue_size=1)
        
        # 取消当前目标的发布者
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
        # 监听move_base目标
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        
        self.previous_invalid_goal = None
        
    def is_point_in_region(self, point):
        """检查点是否在指定区域内"""
        return (self.x_min <= point.x <= self.x_max and 
                self.y_min <= point.y <= self.y_max)
    
    def frontiers_callback(self, markers_msg):
        """过滤前沿点，只保留区域内的点"""
        filtered_markers = MarkerArray()
        
        for marker in markers_msg.markers:
            if marker.type == Marker.POINTS:
                # 创建新标记
                new_marker = Marker()
                new_marker.header = marker.header
                new_marker.ns = marker.ns
                new_marker.id = marker.id
                new_marker.type = marker.type
                new_marker.action = marker.action
                new_marker.pose = marker.pose
                new_marker.scale = marker.scale
                new_marker.color = marker.color
                new_marker.lifetime = marker.lifetime
                
                # 只保留区域内的点
                for i, point in enumerate(marker.points):
                    if self.is_point_in_region(point):
                        new_marker.points.append(point)
                        if i < len(marker.colors):
                            new_marker.colors.append(marker.colors[i])
                
                if new_marker.points:
                    filtered_markers.markers.append(new_marker)
            
            elif marker.type == Marker.SPHERE:
                # 对于球体标记，检查其位置
                if self.is_point_in_region(marker.pose.position):
                    filtered_markers.markers.append(marker)
        
        # 发布过滤后的前沿点
        if filtered_markers.markers:
            self.filtered_frontiers_pub.publish(filtered_markers)
    
    def goal_callback(self, goal_msg):
        """检查导航目标是否在允许区域内，如果不在则取消"""
        goal_point = goal_msg.goal.target_pose.pose.position
        
        if not self.is_point_in_region(goal_point):
            # 避免重复取消同一个目标
            if (self.previous_invalid_goal is None or 
                abs(goal_point.x - self.previous_invalid_goal.x) > 0.1 or
                abs(goal_point.y - self.previous_invalid_goal.y) > 0.1):
                
                rospy.logwarn("目标点 (%.2f, %.2f) 在指定区域外，取消导航!", 
                              goal_point.x, goal_point.y)
                
                # 创建一个取消消息
                cancel_msg = GoalID()
                cancel_msg.stamp = rospy.Time.now()
                cancel_msg.id = goal_msg.goal_id.id
                
                # 发布取消消息
                self.cancel_pub.publish(cancel_msg)
                self.previous_invalid_goal = goal_point

if __name__ == '__main__':
    try:
        explorer = RegionLimitedExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass