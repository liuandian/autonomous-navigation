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
from visualization_msgs.msg import MarkerArray

# 定义状态：初始化
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Initializing system...')

        return 'initialized'

# 定义状态：前沿探索任务
class ExploreFrontier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # 初始化探索区域的地图数据
        self.costmap_msg = OccupancyGrid()
        self.costmap_updates_msg = OccupancyGrid()
        
        # 全局代价地图的订阅器
        self.costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.costmap_updates_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGrid, self.costmap_updates_callback)
        
        # 前沿探索区域的地图数据的发布器
        self.explore_costmap_publisher = rospy.Publisher('/frontier_explore/costmap', OccupancyGrid, queue_size=10)
        self.explore_costmap_updates_publisher = rospy.Publisher('/frontier_explore/costmap_updates', OccupancyGrid, queue_size=10)
        
        # 添加前沿点订阅器 - 监听explore_lite发布的前沿点可视化
        self.frontier_subscriber = rospy.Subscriber('/explore/frontiers', MarkerArray, self.frontier_callback)
        
        # 前沿点计数和阈值设置
        self.frontier_count = 999  # 初始化为一个大数字
        self.frontier_threshold = 3  # 当前沿点数量小于此值时认为探索完成
        
    def execute(self, userdata):
        rospy.loginfo('开始前沿探索任务...')

        # 定期检查前沿点数量和更新掩码地图
        rate = rospy.Rate(0.5)  # 每2秒检查一次
        explore_start_time = rospy.Time.now()
        max_explore_time = rospy.Duration(300)  # 最大探索时间限制为5分钟
        
        rospy.loginfo('开始监控前沿点数量，阈值为%d', self.frontier_threshold)
        
        # 等待前沿探索启动并开始发布前沿点
        rospy.sleep(5)  
        
        while not rospy.is_shutdown():
            if self.costmap_msg.data == [] or self.costmap_updates_msg.data == []:
                rospy.loginfo('等待地图数据...')
                continue
            rospy.loginfo('收到地图数据，开始处理')

            # 更新掩码代价地图
            masked_costmap = self.segment_costmap(self.costmap_msg)
            masked_costmap_updates = self.segment_costmap(self.costmap_updates_msg)
            rospy.loginfo('掩码代价地图已更新')
            
            self.explore_costmap_publisher.publish(masked_costmap)
            self.explore_costmap_updates_publisher.publish(masked_costmap_updates)
            rospy.loginfo('掩码代价地图已发布')
            
            # 检查前沿点数量
            rospy.loginfo('当前前沿点数量: %d (阈值: %d)', self.frontier_count, self.frontier_threshold)
            
            # 如果前沿点数量小于阈值，认为探索完成
            if self.frontier_count <= self.frontier_threshold:
                rospy.loginfo('前沿点数量已低于阈值，探索任务完成')
                return 'succeeded'
            
            # 检查是否超时
            if (rospy.Time.now() - explore_start_time) > max_explore_time:
                rospy.loginfo('探索任务超时，强制完成')
                return 'succeeded'
            
            rate.sleep()
        
        return 'succeeded'

    def costmap_callback(self, msg):
        self.costmap_msg = msg
    
    def costmap_updates_callback(self, msg):
        self.costmap_updates_msg = msg
    
    def frontier_callback(self, msg):
        # 计算有效的前沿点数量（通常每个前沿由多个标记点组成）
        # 只计算TYPE_SPHERE类型的标记，它代表前沿中心点
        frontier_count = 0
        for marker in msg.markers:
            if marker.type == marker.SPHERE:
                frontier_count += 1
        
        self.frontier_count = frontier_count
        
    def segment_costmap(self, costmap_msg):
        # 复制原始代价地图
        masked_costmap = costmap_msg
        
        # 设置不需要探索的区域为障碍物(100)或未知区域(-1)
        width = masked_costmap.info.width
        height = masked_costmap.info.height
        resolution = masked_costmap.info.resolution
        origin_x = masked_costmap.info.origin.position.x
        origin_y = masked_costmap.info.origin.position.y
        
        # 转换实际坐标到地图索引
        x_min = int((2.0 - origin_x) / resolution)
        x_max = int((22.0 - origin_x) / resolution)
        y_min = int((0.0 - origin_y) / resolution)
        y_max = int((11.0 - origin_y) / resolution)
        
        # 限制索引在地图范围内
        x_min = max(0, min(width-1, x_min))
        x_max = max(0, min(width-1, x_max))
        y_min = max(0, min(height-1, y_min))
        y_max = max(0, min(height-1, y_max))
        
        # 标记区域为障碍物
        for y in range(y_min, y_max+1):
            for x in range(x_min, x_max+1):
                idx = y * width + x
                if idx < len(masked_costmap.data):
                    masked_costmap.data[idx] = 100  # 标记为障碍物
                    
        return masked_costmap          

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
    ############先开启launch文件，启动world, navigation, ocr等节点############
    # 初始化ROS节点
    rospy.init_node('task_coordinator')

    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    
    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'NAVIGATE_TO_EXPLORATION_AREA',
                                           'failed':'mission_failed'})
                
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