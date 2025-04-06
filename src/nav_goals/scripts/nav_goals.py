#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose2D

class GoalForwarder2D:
    def __init__(self):
        rospy.init_node('nav_goals', anonymous=True)
        rospy.sleep(1)  # 🔁 确保 /clock 开始发布时间
        rospy.Subscriber('custom_goal_pose2d', Pose2D, self.callback)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.loginfo("2D Nav Goal  node started.")
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo(f"Received Pose2D: x={msg.x}, y={msg.y}, theta={msg.theta}")

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # 注意根据你的TF框架选择 map/base_link等

        goal.pose.position.x = msg.x
        goal.pose.position.y = msg.y
        goal.pose.position.z = 0.0

        # 将 yaw (theta) 转换为四元数
        quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        self.pub.publish(goal)
        rospy.loginfo("Published goal to /move_base_simple/goal.")

if __name__ == '__main__':
    try:
        GoalForwarder2D()
    except rospy.ROSInterruptException:
        pass