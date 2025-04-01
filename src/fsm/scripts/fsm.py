#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch

# 定义状态：初始化
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Initializing system...')

        return 'initialized'

# 定义状态：导航到探索区域
class NavigateToExplorationArea(smach.State):
    '''使用move_base进行导航'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" # 不确定
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置导航目标位置
        goal.target_pose.pose.position.x = 23.0
        goal.target_pose.pose.position.y = 19.0
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo('向目标点导航中...')
        self.client.send_goal(goal)
        
        # 等待导航完成并获取结果
        self.client.wait_for_result()
        result = self.client.get_state()
        
        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('成功到达目标点')
            return 'succeeded'
        elif result == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo('导航被取消')
            return 'preempted'
        else:
            rospy.loginfo('导航失败')
            return 'failed'

# 定义状态：前沿探索任务
class ExploreFrontier(smach.State):
    '''rostopic pub /ocr_trigger std_msgs/Bool "data: true" -1'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.ocr_trigger_publisher = rospy.Publisher('/ocr_trigger', Bool, queue_size=10)
        self.frontier_submap_subscriber = rospy.Subscriber('/frontier_submap', PoseStamped, self.submap_callback)
        self.detected_boxes_subscriber = rospy.Subscriber('/detected_boxes', PoseArray, self.box_callback)
        
    def execute(self, userdata):
        rospy.loginfo('执行任务一...')
        # 发布触发消息
        trigger_msg = Bool()
        trigger_msg.data = True
        self.ocr_trigger_publisher.publish(trigger_msg)
        rospy.loginfo('触发消息已发布')
        # 等待OCR处理完成
        rospy.sleep(5)  # 模拟OCR处理时间
        return 'succeeded'
    
    def box_callback(self, msg):
        rospy.loginfo('检测到盒子，位置：%s', msg.poses[0].position)
        # 处理检测到的盒子位置
        # 这里可以添加更多的处理逻辑，例如保存位置、发送到其他节点等
        # self.pub.publish(msg)  # 发布检测到的盒子位置
    
# 定义状态：盒子检测任务
class DetectBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.box_subscriber = rospy.Subscriber('/detected_boxes', PoseArray, self.box_callback)
        
    def execute(self, userdata):
        rospy.loginfo('执行盒子检测任务...')
        # 这里可以添加更多的处理逻辑，例如保存位置、发送到其他节点等
        return 'succeeded'
    
    def box_callback(self, msg):
        rospy.loginfo('检测到盒子，位置：%s', msg.poses[0].position)
        # 处理检测到的盒子位置
        # 这里可以添加更多的处理逻辑，例如保存位置、发送到其他节点等
        # self.pub.publish(msg)  # 发布检测到的盒子位置

# 定义状态：桥检测任务  
class DetectBridge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo('执行任务二...')
        # 任务二的代码
        return 'succeeded'
    
# 定义状态：导航到目标
class NavigateToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo('执行任务三...')
        # 任务三的代码
        return 'succeeded'


# 主函数
def main():
    # # 启动launch文件
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch_file = "./src/fsm/launch/final.launch"
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    # launch.start()
    # rospy.loginfo("Launch file started")
    # # 等待ROS系统准备就绪
    # rospy.sleep(10)
    
    # 初始化ROS节点
    rospy.init_node('task_coordinator')

    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    
    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                            #    transitions={'initialized':'NAVIGATE_TO_GOAL',
                               transitions={'initialized':'NAVIGATE_TO_GOAL',
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('NAVIGATE_TO_GOAL', NavigateToGoal(),
                               transitions={'succeeded':'TASK_ONE', 
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('TASK_ONE', ExploreFrontier(),
                               transitions={'succeeded':'TASK_TWO', 
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('TASK_TWO', DetectBridge(),
                               transitions={'succeeded':'TASK_THREE', 
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('TASK_THREE', NavigateToGoal(),
                               transitions={'succeeded':'mission_completed', 
                                           'failed':'mission_failed'})
    
    # 创建状态机的可视化工具
    sis = smach_ros.IntrospectionServer('task_coordinator', sm, '/TASK_COORDINATOR')
    sis.start()
    
    # 执行状态机
    outcome = sm.execute()
    
    rospy.loginfo('状态机执行完毕，结果：%s' % outcome)
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()