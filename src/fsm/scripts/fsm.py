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

        if not self.box_positions or len(self.box_positions) == 0:
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
        # 添加输出键bridge_entrance和bridge_exit
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'failed'],
                            output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.bridge_detection_trigger_publisher = rospy.Publisher('/bridge_detection_trigger', Bool, queue_size=10)
        self.bridge_pose_subscriber = rospy.Subscriber('/detected_bridges', PoseStamped, self.bridge_callback)
        self.bridge_entrance = None  # 桥入口坐标
        self.detection_complete = False  # 检测完成标志
        
    def execute(self, userdata):
        # 重置检测状态
        self.bridge_entrance = None
        self.detection_complete = False
        
        rospy.loginfo('执行桥检测任务...')
        # 发布桥检测触发消息
        bridge_detection_trigger_msg = Bool()
        bridge_detection_trigger_msg.data = True
        self.bridge_detection_trigger_publisher.publish(bridge_detection_trigger_msg)
        rospy.loginfo('桥检测触发消息已发布')
        
        # 等待桥梁检测结果
        timeout = rospy.Duration(15)  # 设置超时时间为15秒
        start_time = rospy.Time.now()
        
        while not self.detection_complete and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.5)
            
        if self.bridge_entrance is None:
            rospy.loginfo('未检测到桥梁或检测超时')
            return 'failed'
        
        # 计算出口坐标（入口坐标y值减3）
        bridge_exit = PoseStamped()
        bridge_exit.header = self.bridge_entrance.header
        bridge_exit.pose.position.x = self.bridge_entrance.pose.position.x
        bridge_exit.pose.position.y = self.bridge_entrance.pose.position.y - 3.0
        bridge_exit.pose.orientation = self.bridge_entrance.pose.orientation
        
        # 将入口和出口坐标保存到userdata中传递给下一个状态
        userdata.bridge_entrance_out = self.bridge_entrance
        userdata.bridge_exit_out = bridge_exit
        
        rospy.loginfo('桥梁检测成功')
        rospy.loginfo('桥入口坐标: x=%f, y=%f', 
                     self.bridge_entrance.pose.position.x, 
                     self.bridge_entrance.pose.position.y)
        rospy.loginfo('桥出口坐标: x=%f, y=%f', 
                     bridge_exit.pose.position.x, 
                     bridge_exit.pose.position.y)
        
        return 'succeeded'
    
    def bridge_callback(self, msg):
        rospy.loginfo('检测到桥，位置：x=%f, y=%f' % (msg.pose.position.x, msg.pose.position.y))
        self.bridge_entrance = msg
        self.detection_complete = True

# 定义状态：导航到桥入口
class NavigateToBridgeEntrance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_entrance_in'],
                           output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, userdata):
        # 从userdata中获取桥梁入口坐标
        bridge_entrance = userdata.bridge_entrance_in
        
        if bridge_entrance is None:
            rospy.loginfo('缺少桥梁入口坐标信息')
            return 'failed'
        
        rospy.loginfo('开始导航到桥入口...')
        
        # 导航到桥入口
        rospy.loginfo('导航到桥入口: x=%f, y=%f', 
                     bridge_entrance.pose.position.x, 
                     bridge_entrance.pose.position.y)
        
        goal = MoveBaseGoal()
        goal.target_pose = bridge_entrance
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_state()
        
        if result != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('导航到桥入口失败')
            return 'failed'
        
        rospy.loginfo('已成功到达桥入口')
        # 传递坐标到下一个状态
        userdata.bridge_entrance_out = bridge_entrance
        userdata.bridge_exit_out = userdata.bridge_exit_in
        
        return 'succeeded'

# 定义状态：开桥并导航过桥
class OpenBridgeAndNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_exit_in'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.open_bridge_publisher = rospy.Publisher('/open_bridge', Bool, queue_size=10)
        
    def execute(self, userdata):
        # 从userdata中获取桥梁出口坐标
        bridge_exit = userdata.bridge_exit_in
        
        if bridge_exit is None:
            rospy.loginfo('缺少桥梁出口坐标信息')
            return 'failed'
        
        # 发送开桥指令
        open_bridge_msg = Bool()
        open_bridge_msg.data = True
        self.open_bridge_publisher.publish(open_bridge_msg)
        rospy.loginfo('已发送开桥指令')
        
        # 给桥梁足够的打开时间
        rospy.sleep(3.0)
        
        # 导航到桥出口（过桥）
        rospy.loginfo('开始过桥，导航到桥出口: x=%f, y=%f', 
                     bridge_exit.pose.position.x, 
                     bridge_exit.pose.position.y)
        
        goal = MoveBaseGoal()
        goal.target_pose = bridge_exit
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_state()
        
        if result != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('过桥失败')
            return 'failed'
        
        rospy.loginfo('成功过桥')
        return 'succeeded'

# 定义状态：导航到目标并启动OCR
class NavigateToGoalAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # 四个目标盒子的坐标：(6,0,3)，(10,0,3)，(14,0,3)，(18,0,3)
        self.goals = [
            (6.0, 0.0, 3.0),
            (10.0, 0.0, 3.0),
            (14.0, 0.0, 3.0),
            (18.0, 0.0, 3.0)
        ]
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.ocr_trigger_publisher = rospy.Publisher('/ocr_trigger', Bool, queue_size=10)

        # 一旦匹配到目标盒子，OCR节点会发布/cmd_stop
        self.cmd_stop_subscriber = rospy.Subscriber('/cmd_stop', Bool, self.cmd_stop_callback)
        self.cmd_stop_received = False  # 标记是否收到停止指令
        
    def execute(self, userdata):
        rospy.loginfo('执行导航到目标任务...')
        # 遍历目标坐标，依次导航
        for goal_position in self.goals:
            self.goal.target_pose.pose.position.x = goal_position[0]
            self.goal.target_pose.pose.position.y = goal_position[1]
            self.goal.target_pose.pose.orientation.w = goal_position[2]
            
            rospy.loginfo('导航到目标位置: x=%f, y=%f', 
                          goal_position[0], goal_position[1])
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            result = self.client.get_state()
            
            if result == actionlib.GoalStatus.SUCCEEDED:
                # 启动OCR处理
                ocr_trigger_msg = Bool()
                ocr_trigger_msg.data = True
                self.ocr_trigger_publisher.publish(ocr_trigger_msg)
                rospy.sleep(2)
                # 等待OCR处理完成
                if self.cmd_stop_received:
                    rospy.loginfo('OCR任务完成，停止导航')
                    self.cmd_stop_received = False
                    break
                else:
                    rospy.loginfo('OCR任务尚未完成，继续导航')
            else:
                rospy.loginfo('导航到目标位置失败')
                return 'failed'

        rospy.loginfo('所有目标位置导航完成')

        return 'succeeded'
    
    def cmd_stop_callback(self, msg):
        if msg.data== True:
            rospy.loginfo('收到停止指令，OCR任务完成')
            self.cmd_stop_received = True
        else:
            rospy.loginfo('未收到停止指令，继续等待')

# 主函数
def main():
    # 初始化ROS节点
    rospy.init_node('task_coordinator')

    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    
    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'NAVIGATE_TO_EXPLORATION_AREA',
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('NAVIGATE_TO_EXPLORATION_AREA', NavigateToExplorationArea(), 
                               transitions={'succeeded':'EXPLORE_FRONTIER', 
                                           'failed':'mission_failed',
                                           'preempted':'mission_failed'})
        
        smach.StateMachine.add('EXPLORE_FRONTIER', ExploreFrontier(),
                               transitions={'succeeded':'DETECT_BOX_POSE', 
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('DETECT_BOX_POSE', DetectBoxPose(), 
                               transitions={'succeeded':'NAVIGATE_TO_BOX_AND_OCR', 
                                           'failed':'mission_failed'},
                               remapping={'box_positions_out':'box_positions'})
        
        smach.StateMachine.add('NAVIGATE_TO_BOX_AND_OCR', NavigateToBoxAndOCR(),
                               transitions={'succeeded':'DETECT_BRIDGE', 
                                           'failed':'mission_failed'},
                               remapping={'box_positions_in':'box_positions'})
        
        smach.StateMachine.add('DETECT_BRIDGE', DetectBridge(),
                               transitions={'succeeded':'NAVIGATE_TO_BRIDGE_ENTRANCE', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_entrance_out':'bridge_entrance',
                                          'bridge_exit_out':'bridge_exit'})
        
        smach.StateMachine.add('NAVIGATE_TO_BRIDGE_ENTRANCE', NavigateToBridgeEntrance(),
                               transitions={'succeeded':'OPEN_BRIDGE_AND_NAVIGATE', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_entrance_in':'bridge_entrance',
                                          'bridge_entrance_out':'bridge_entrance',
                                          'bridge_exit_out':'bridge_exit'})
        
        smach.StateMachine.add('OPEN_BRIDGE_AND_NAVIGATE', OpenBridgeAndNavigate(),
                               transitions={'succeeded':'NAVIGATE_TO_GOAL_AND_OCR', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_exit_in':'bridge_exit'})
        
        smach.StateMachine.add('NAVIGATE_TO_GOAL_AND_OCR', NavigateToGoalAndOCR(),
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