import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Twist
import tf2_ros
import tf2_geometry_msgs
import tf_conversions

class StateMachine:
    def __init__(self):
        rospy.init_node("state_manager")

        self.state_list = ["init", "boxes_count", "cross_bridge", "find_goal"]
        self.current_state = None

        self.state_topics = {
            "init": "/do_init",
            "boxes_count": "/do_boxes_count",
            "cross_bridge": "/do_cross_bridge",
            "find_goal": "/do_find_goal"
        }

        self.publishers = {
            state: rospy.Publisher(topic, Bool, queue_size=1)
            for state, topic in self.state_topics.items()
        }

        self.state_handlers = {
            "init": self.handle_init,
            "boxes_count": self.handle_boxes_count,
            "cross_bridge": self.handle_cross_bridge,
            "find_goal": self.handle_find_goal,
        }

        for state in self.state_list:
            rospy.Subscriber(f"/start_{state}", Bool, self.make_callback(state))
        self.goal_reached = False

        # Initialization
        self.robot_frame = "base_link"
        self.map_frame = "map"
        self.world_frame = "world"
        self.current_state = "init"
    def make_callback(self, target_state):
        def callback(msg):
            if msg.data and self.current_state != target_state:
                rospy.loginfo(f"[状态切换] {self.current_state} -> {target_state}")
                self.current_state = target_state
        return callback

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(5, f"当前状态: {self.current_state}")
            for state in self.state_list:
                self.publishers[state].publish(Bool(data=(state == self.current_state)))

            if self.current_state and self.current_state in self.state_handlers:
                self.state_handlers[self.current_state]()

            rate.sleep()

    def handle_init(self):
        rospy.loginfo_throttle(5, "[init]")


    def handle_boxes_count(self):
        rospy.loginfo_throttle(5, "[boxes_count] ")

    def handle_cross_bridge(self):
        rospy.loginfo_throttle(5, "[cross_bridge] ")

    def handle_find_goal(self):
        rospy.loginfo_throttle(5, "[find_goal] ")


        
if __name__ == "__main__":
    sm = StateMachine()
    sm.run()
    rospy.spin()
