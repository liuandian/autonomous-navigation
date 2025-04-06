import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class StateMachine:
    def __init__(self):
        rospy.init_node("state_manager")

        # 初始化变量
        self.number9_pose = None

        # 订阅 /gazebo/model_states 话题
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

    def model_states_callback(self, msg):
        """
        回调函数，用于解析 /gazebo/model_states 中的 number9 位置信息
        """
        try:
            # 获取模型名称的索引
            index = msg.name.index("number9")
            # 获取对应的位置信息
            self.number9_pose = msg.pose[index]
            rospy.loginfo(f"Number9 Position: x={self.number9_pose.position.x}, y={self.number9_pose.position.y}, z={self.number9_pose.position.z}")
        except ValueError:
            rospy.logwarn("Model 'number9' not found in /gazebo/model_states")
        except Exception as e:
            rospy.logerr(f"Error in model_states_callback: {e}")

    def run(self):
        """
        主循环
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.number9_pose:
                rospy.loginfo_throttle(5, f"Current Number9 Position: x={self.number9_pose.position.x}, y={self.number9_pose.position.y}, z={self.number9_pose.position.z}")
            rate.sleep()

if __name__ == "__main__":
    sm = StateMachine()
    sm.run()