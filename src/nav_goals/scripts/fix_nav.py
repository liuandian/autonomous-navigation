#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class FixedPathNavigator:
    def __init__(self):
        rospy.init_node('fixed_path_navigator', anonymous=True)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        self.path = [
            (20.0, 0.0, 0.0),
            (21.0, -21.6, -1.57),
            (18.9, -21.6, -3.14),
            (18.9, -12.3, 1.57),
            (18.9, -3.0, 1.57),
            (14.5, -3.0, 3.14),
            (14.5, -12.3, -1.57),
            (14.5, -21.6, -1.57),
            (10.0, -21.6, -3.14),
            (10.0, -12.3, 1.57),
            (10.0, -3.0, -1.57)
        ]

        self.current_index = 0
        rospy.sleep(2.0)  # 等待时钟、发布器初始化
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.path):
            rospy.loginfo("✅ 所有目标点均已完成导航。")
            return

        x, y, theta = self.path[self.current_index]
        rospy.loginfo(f"➡️ 发布目标 {self.current_index + 1}/{len(self.path)}: ({x}, {y}, {theta})")

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.goal_pub.publish(goal)

    def result_callback(self, msg):
        status = msg.status.status
        if status == 3:  # SUCCEEDED
            rospy.loginfo("✅ 到达当前目标点。")
            self.current_index += 1
            rospy.sleep(0.5)
            self.send_next_goal()
        elif status in [4, 5, 9]:  # ABORTED, REJECTED, LOST
            rospy.logwarn("⚠️ 到达目标失败，尝试下一个目标。")
            self.current_index += 1
            rospy.sleep(0.5)
            self.send_next_goal()
        else:
            # 其他状态不处理
            pass

if __name__ == '__main__':
    try:
        FixedPathNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
