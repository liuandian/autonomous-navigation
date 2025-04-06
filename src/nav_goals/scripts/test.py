#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/custom_goal_pose2d', Pose2D, queue_size=10)


    goal = Pose2D()
    goal.x = 2.0  # 目标x坐标
    goal.y = 3.0  # 目标y坐标
    goal.theta = 1.57  # 目标朝向角度（弧度）

    rospy.loginfo(f"Publishing goal: x={goal.x}, y={goal.y}, theta={goal.theta}")
    pub.publish(goal)



if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
