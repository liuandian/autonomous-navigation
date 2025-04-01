#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
from nav_msgs.msg import OccupancyGrid

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
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # 初始化探索区域的地图数据
        self.costmap_msg = OccupancyGrid()
        self.costmap_updates_msg = OccupancyGrid()
        
        # 发布探索区域的地图数据
        self.explore_costmap_publisher = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=10)
        self.explore_costmap_updates_publisher = rospy.Publisher('/move_base/global_costmap/costmap_updates', OccupancyGrid, queue_size=10)
        
    def execute(self, userdata):
        # TODO:初始化探索区域的地图数据，costmap_msg来源于/map, costmap_updates_msg通常来源于/map_updates
        self.costmap_msg = ...
        self.costmap_updates_msg = ...

        rospy.loginfo('执行前沿探索任务...')
        # 这里可以添加前沿探索的代码
        # 例如，发布探索区域的地图数据
        self.explore_costmap_publisher.publish(self.costmap_msg)
        self.explore_costmap_updates_publisher.publish(self.costmap_updates_msg)
    
# 定义状态：盒子位置检测任务
class DetectBoxPose(smach.State):
    def __init__(self):
        # 添加输出键box_positions
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'failed'],
                            output_keys=['box_positions_out'])
        self.box_detection_trigger_publisher = rospy.Publisher('/box_detection_trigger', Bool, queue_size=10)
        self.box_pose_subscriber = rospy.Subscriber('/detected_boxes', PoseArray, self.box_callback)
        self.box_positions = []  # 用于存储检测到的盒子位置

    def execute(self, userdata):
        # 重置盒子位置列表
        self.box_positions = []

        rospy.loginfo('执行盒子检测任务...')
        # 发布盒子检测触发消息
        box_detection_trigger_msg = Bool()
        box_detection_trigger_msg.data = True
        self.box_detection_trigger_publisher.publish(box_detection_trigger_msg)
        rospy.loginfo('盒子检测触发消息已发布')
        # 等待盒子检测结果
        rospy.sleep(5)

        # 将盒子位置保存到userdata中传递给下一个状态
        userdata.box_positions_out = self.box_positions

        if not self.box_positions:
            rospy.loginfo('未检测到盒子')
            return 'failed'
        return 'succeeded'
    
    def box_callback(self, msg):
        rospy.loginfo('检测到多个盒子，数量：%d' % len(msg.poses))
        # 处理检测到的盒子位置
        for pose in msg.poses:
            rospy.loginfo('盒子位置：x=%f, y=%f' % (pose.position.x, pose.position.y))
            # 保存盒子位置保存到一个列表中
            self.box_positions.append(pose)

# 定义状态：导航至每个盒子并启动OCR
class NavigateToBoxAndOCR(smach.State):
    '''rostopic pub /ocr_trigger std_msgs/Bool "data: true" -1'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['box_positions_in'])
        self.ocr_trigger_publisher = rospy.Publisher('/ocr_trigger', Bool, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, userdata):
        # 从userdata中获取盒子位置
        box_positions = userdata.box_positions_in
        if not box_positions:
            rospy.loginfo('没有盒子位置可供导航')
            return 'failed'
        rospy.loginfo('导航到%d个盒子并启动OCR...', len(box_positions))
        
        # 对每个盒子进行导航和OCR
        for box_pose in box_positions:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = box_pose
            
            rospy.loginfo('导航到盒子位置: x=%f, y=%f', 
                          box_pose.position.x, box_pose.position.y)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            
            # 启动OCR处理
            ocr_trigger_msg = Bool()
            ocr_trigger_msg.data = True
            self.ocr_trigger_publisher.publish(ocr_trigger_msg)
            rospy.sleep(2)  # 给OCR处理一些时间
            
        return 'succeeded'

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
        
        smach.StateMachine.add('DETECT_BOX_POSE', DetectBoxPose(), 
                            transitions={'succeeded':'NAVIGATE_TO_BOX_AND_OCR', 
                                        'failed':'mission_failed'},
                            remapping={'box_positions_out':'box_positions'})
        
        smach.StateMachine.add('NAVIGATE_TO_BOX_AND_OCR', NavigateToBoxAndOCR(),
                            transitions={'succeeded':'TASK_THREE', 
                                        'failed':'mission_failed'},
                            remapping={'box_positions_in':'box_positions'})
        
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