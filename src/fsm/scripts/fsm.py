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
from map_msgs.msg import OccupancyGridUpdate  # 添加此行，导入正确的消息类型
from visualization_msgs.msg import MarkerArray

# ====================== 全局配置参数 ======================
class Config:
    # 话题名称配置
    TOPICS = {
        # 代价地图相关话题
        'GLOBAL_COSTMAP': '/move_base/global_costmap/costmap',
        'GLOBAL_COSTMAP_UPDATES': '/move_base/global_costmap/costmap_updates',
        'EXPLORE_COSTMAP': '/frontier_explore/costmap',
        'EXPLORE_COSTMAP_UPDATES': '/frontier_explore/costmap_updates',
        'FRONTIERS': '/explore/frontiers',
        
        # 检测触发和结果话题
        'BOX_DETECTION_TRIGGER': '/box_detection_trigger',
        'DETECTED_BOXES': '/detected_boxes',
        'BRIDGE_DETECTION_TRIGGER': '/bridge_detection_trigger',
        'DETECTED_BRIDGES': '/detected_bridges',
        'OCR_TRIGGER': '/ocr_trigger',
        'CMD_STOP': '/cmd_stop',
        'OPEN_BRIDGE': '/open_bridge',
        
        # 导航相关话题
        'MOVE_BASE': 'move_base',
    }
    
    # 超时设置(秒)
    TIMEOUTS = {
        'INIT': 10.0,                # 初始化超时
        'EXPLORE': 300.0,            # 探索任务超时
        'BOX_DETECTION': 15.0,       # 盒子检测超时
        'BRIDGE_DETECTION': 15.0,    # 桥梁检测超时
        'NAVIGATION': 60.0,          # 导航超时
        'OCR_PROCESSING': 5.0,       # OCR处理超时
        'BRIDGE_OPEN': 3.0,          # 桥梁打开等待时间
    }
    
    # 地图探索区域设置
    MAP_AREA = {
        'X_MIN': 2.0,    # 地图X坐标最小值
        'X_MAX': 22.0,   # 地图X坐标最大值
        'Y_MIN': 0.0,    # 地图Y坐标最小值
        'Y_MAX': 11.0,   # 地图Y坐标最大值
    }
    
    # 探索相关设置
    EXPLORE = {
        'FRONTIER_THRESHOLD': 3,     # 前沿点数量阈值，低于此值认为探索完成
        'CHECK_RATE': 0.5,           # 检查频率(Hz)
    }
    
    # 桥梁相关设置
    BRIDGE = {
        'EXIT_Y_OFFSET': -3.0,       # 桥出口相对于入口的Y轴偏移
    }
    
    # 目标位置坐标列表 (x, y, orientation.w)
    GOALS = [
        (6.0, 0.0, 3.0),
        (10.0, 0.0, 3.0),
        (14.0, 0.0, 3.0),
        (18.0, 0.0, 3.0)
    ]

# 定义状态：初始化
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.timeout = Config.TIMEOUTS['INIT']

    def execute(self, userdata):
        rospy.loginfo('初始化系统...')
        
        try:
            # 在此添加初始化检查逻辑
            # 例如检查各个必要服务是否可用
            
            return 'initialized'
        except Exception as e:
            rospy.logerr('初始化过程出错: %s', str(e))
            return 'failed'

# 定义状态：前沿探索任务
class ExploreFrontier(smach.State):
    def __init__(self):
        import threading
        self.map_lock = threading.Lock()
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # 初始化探索区域的地图数据
        self.costmap_msg = OccupancyGrid()
        self.costmap_updates_msg = OccupancyGridUpdate()
        
        # 全局代价地图的订阅器
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['GLOBAL_COSTMAP'], 
            OccupancyGrid, 
            self.costmap_callback
        )
        self.costmap_updates_subscriber = rospy.Subscriber(
            Config.TOPICS['GLOBAL_COSTMAP_UPDATES'], 
            OccupancyGridUpdate,
            self.costmap_updates_callback
        )
        
        # 前沿探索区域的地图数据的发布器
        self.explore_costmap_publisher = rospy.Publisher(
            Config.TOPICS['EXPLORE_COSTMAP'], 
            OccupancyGrid, 
            queue_size=10
        )
        self.explore_costmap_updates_publisher = rospy.Publisher(
            Config.TOPICS['EXPLORE_COSTMAP_UPDATES'], 
            OccupancyGridUpdate,  # 使用正确的消息类型
            queue_size=10
        )        
        # 添加前沿点订阅器 - 监听explore_lite发布的前沿点可视化
        self.frontier_subscriber = rospy.Subscriber(
            Config.TOPICS['FRONTIERS'], 
            MarkerArray, 
            self.frontier_callback
        )
        
        # 前沿点计数和阈值设置
        self.frontier_count = 999  # 初始化为一个大数字
        self.frontier_threshold = Config.EXPLORE['FRONTIER_THRESHOLD']  
        self.max_explore_time = Config.TIMEOUTS['EXPLORE']  
        
    def execute(self, userdata):
        rospy.loginfo('开始前沿探索任务...')

        # 定期检查前沿点数量和更新掩码地图
        rate = rospy.Rate(Config.EXPLORE['CHECK_RATE'])  
        explore_start_time = rospy.Time.now()
        
        rospy.loginfo('开始监控前沿点数量，阈值为%d', self.frontier_threshold)
        
        # 等待前沿探索启动并开始发布前沿点
        rospy.loginfo('等待前沿探索启动...')
        # rospy.sleep(5)  # 等待5秒钟，确保前沿点开始发布

        while not rospy.is_shutdown():
            with self.map_lock:
                # 使用锁保护地图数据的访问
                if self.costmap_msg.data == [] or self.costmap_updates_msg.data == []:
                    rospy.loginfo('等待地图数据...')
                    rate.sleep()
                    continue
                rospy.loginfo('收到地图数据，开始处理')

                # 更新掩码代价地图
                masked_costmap = self.segment_costmap(self.costmap_msg)
                masked_costmap_updates = self.segment_costmap_update(self.costmap_updates_msg)
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
                if (rospy.Time.now() - explore_start_time) > self.max_explore_time:
                    rospy.loginfo('探索任务超时，强制完成')
                    return 'succeeded'
                
                rate.sleep()
        
        return 'succeeded'

    def costmap_callback(self, msg):
        with self.map_lock:
            # 处理全局代价地图
            self.costmap_msg = msg
            self.last_complete_map = msg
    
    def costmap_updates_callback(self, msg):
        with self.map_lock:
            self.costmap_updates_msg = msg
    
    def frontier_callback(self, msg):
        with self.map_lock:
            # 计算有效的前沿点数量（通常每个前沿由多个标记点组成）
            # 只计算TYPE_SPHERE类型的标记，它代表前沿中心点
            frontier_count = 0
            for marker in msg.markers:
                if marker.type == marker.SPHERE:
                    frontier_count += 1
            
            self.frontier_count = frontier_count
        
    def segment_costmap(self, costmap_msg):
        """处理代价地图，设置不需要探索的区域为障碍物或未知区域，数学变换有待确认"""
        import copy
        # 复制原始代价地图
        masked_costmap = copy.deepcopy(costmap_msg)
        
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
        data_list = list(masked_costmap.data)  # 转换为列表
        for y in range(y_min, y_max+1):
            for x in range(x_min, x_max+1):
                idx = y * width + x
                if idx < len(masked_costmap.data):
                    data_list[idx] = 100  # 标记为障碍物
        masked_costmap.data = tuple(data_list)  # 转换回元组
        return masked_costmap          

    def segment_costmap_update(self, update_msg):
        """处理代价地图更新，设置不需要探索的区域为障碍物或未知区域，数学变换有待确认"""
        import copy
        masked_update = copy.deepcopy(update_msg)
        
        # 获取更新区域的信息
        width = masked_update.width
        height = masked_update.height
        x_start = masked_update.x
        y_start = masked_update.y
        
        # 如果没有数据，直接返回
        if not masked_update.data:
            return masked_update
        
        # 获取全局代价地图的信息（用于坐标转换）
        if not hasattr(self, 'last_complete_map') or not self.last_complete_map.info:
            rospy.logwarn('没有全局代价地图信息，无法处理更新')
            return masked_update
        
        # 从完整地图中获取分辨率和原点位置
        resolution = self.last_complete_map.info.resolution
        origin_x = self.last_complete_map.info.origin.position.x
        origin_y = self.last_complete_map.info.origin.position.y
        
        # 获取感兴趣区域的边界（实际世界坐标）
        x_min_world = Config.MAP_AREA['X_MIN']
        x_max_world = Config.MAP_AREA['X_MAX']
        y_min_world = Config.MAP_AREA['Y_MIN']
        y_max_world = Config.MAP_AREA['Y_MAX']
        
        # 计算更新区域在实际世界中的绝对坐标
        update_origin_x = origin_x + x_start * resolution
        update_origin_y = origin_y + y_start * resolution
        
        # 将世界坐标转换为局部更新区域内的索引
        x_min_idx = int((x_min_world - update_origin_x) / resolution)
        x_max_idx = int((x_max_world - update_origin_x) / resolution)
        y_min_idx = int((y_min_world - update_origin_y) / resolution)
        y_max_idx = int((y_max_world - update_origin_y) / resolution)
        
        # 限制索引在更新区域范围内
        x_min_idx = max(0, min(width-1, x_min_idx))
        x_max_idx = max(0, min(width-1, x_max_idx))
        y_min_idx = max(0, min(height-1, y_min_idx))
        y_max_idx = max(0, min(height-1, y_max_idx))
        
        # 标记区域为障碍物
        data_list = list(masked_update.data)  # 转换为列表
        for y in range(y_min_idx, y_max_idx+1):
            for x in range(x_min_idx, x_max_idx+1):
                idx = y * width + x
                if 0 <= idx < len(masked_update.data):
                    data_list[idx] = 100  # 标记为障碍物
        masked_update.data = tuple(data_list)
        
        return masked_update

# 定义状态：盒子位置检测任务
class DetectBoxPose(smach.State):
    def __init__(self):
        # 添加输出键box_positions
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'failed'],
                            output_keys=['box_positions_out'])
        self.box_detection_trigger_publisher = rospy.Publisher(
            Config.TOPICS['BOX_DETECTION_TRIGGER'], 
            Bool, 
            queue_size=10
        )
        self.box_pose_subscriber = rospy.Subscriber(
            Config.TOPICS['DETECTED_BOXES'], 
            PoseArray, 
            self.box_callback
        )
        self.box_positions = []  # 用于存储检测到的盒子位置
        self.detection_timeout = Config.TIMEOUTS['BOX_DETECTION']

    def execute(self, userdata):
        # 重置盒子位置列表
        self.box_positions = []

        rospy.loginfo('执行盒子检测任务...')
        try:
            # 发布盒子检测触发消息
            box_detection_trigger_msg = Bool()
            box_detection_trigger_msg.data = True
            self.box_detection_trigger_publisher.publish(box_detection_trigger_msg)
            rospy.loginfo('盒子检测触发消息已发布')
            
            # 等待盒子检测结果
            start_time = rospy.Time.now()
            rate = rospy.Rate(2)  # 2Hz检查频率
            
            while (rospy.Time.now() - start_time) < self.detection_timeout:
                if self.box_positions:
                    break
                rate.sleep()
                
            # 将盒子位置保存到userdata中传递给下一个状态
            userdata.box_positions_out = self.box_positions

            if not self.box_positions or len(self.box_positions) == 0:
                rospy.logwarn('未检测到盒子或检测超时')
                return 'failed'
                
            rospy.loginfo('成功检测到%d个盒子', len(self.box_positions))
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('盒子检测错误: %s', str(e))
            return 'failed'
    
    def box_callback(self, msg):
        rospy.loginfo('检测到多个盒子，数量：%d', len(msg.poses))
        # 处理检测到的盒子位置
        self.box_positions = []  # 清空之前的结果
        for pose in msg.poses:
            rospy.loginfo('盒子位置：x=%.2f, y=%.2f', pose.position.x, pose.position.y)
            # 保存盒子位置到列表中
            self.box_positions.append(pose)

# 定义状态：导航至每个盒子并启动OCR
class NavigateToBoxAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'], 
                           input_keys=['box_positions_in'])
        self.ocr_trigger_publisher = rospy.Publisher(
            Config.TOPICS['OCR_TRIGGER'], 
            Bool, 
            queue_size=10
        )
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        
    def execute(self, userdata):
        # 从userdata中获取盒子位置
        box_positions = userdata.box_positions_in
        if not box_positions:
            rospy.logwarn('没有盒子位置可供导航')
            return 'failed'
            
        rospy.loginfo('导航到%d个盒子并启动OCR...', len(box_positions))
        
        # 对每个盒子进行导航和OCR
        for i, box_pose in enumerate(box_positions):
            try:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = box_pose
                
                rospy.loginfo('[%d/%d] 导航到盒子位置: x=%.2f, y=%.2f', 
                              i+1, len(box_positions),
                              box_pose.position.x, box_pose.position.y)
                              
                self.client.send_goal(goal)
                
                # 等待导航结果，添加超时
                if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                    rospy.logwarn('导航到盒子超时')
                    continue
                
                if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                    rospy.logwarn('导航到盒子失败，尝试下一个')
                    continue
                
                # 启动OCR处理
                ocr_trigger_msg = Bool()
                ocr_trigger_msg.data = True
                self.ocr_trigger_publisher.publish(ocr_trigger_msg)
                rospy.loginfo('OCR触发消息已发布，等待处理...')
                rospy.sleep(self.ocr_timeout)  # 给OCR处理一些时间
                
            except Exception as e:
                rospy.logerr('处理盒子时发生错误: %s', str(e))
                
        return 'succeeded'

# 定义状态：桥检测任务  
class DetectBridge(smach.State):
    def __init__(self):
        # 添加输出键bridge_entrance和bridge_exit
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'failed'],
                            output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.bridge_detection_trigger_publisher = rospy.Publisher(
            Config.TOPICS['BRIDGE_DETECTION_TRIGGER'], 
            Bool, 
            queue_size=10
        )
        self.bridge_pose_subscriber = rospy.Subscriber(
            Config.TOPICS['DETECTED_BRIDGES'], 
            PoseStamped, 
            self.bridge_callback
        )
        self.bridge_entrance = None  # 桥入口坐标
        self.detection_complete = False  # 检测完成标志
        self.detection_timeout = Config.TIMEOUTS['BRIDGE_DETECTION']
        
    def execute(self, userdata):
        # 重置检测状态
        self.bridge_entrance = None
        self.detection_complete = False
        
        rospy.loginfo('执行桥检测任务...')
        try:
            # 发布桥检测触发消息
            bridge_detection_trigger_msg = Bool()
            bridge_detection_trigger_msg.data = True
            self.bridge_detection_trigger_publisher.publish(bridge_detection_trigger_msg)
            rospy.loginfo('桥检测触发消息已发布')
            
            # 等待桥梁检测结果
            timeout = rospy.Duration(self.detection_timeout)
            start_time = rospy.Time.now()
            
            rate = rospy.Rate(2)  # 2Hz检查频率
            while not self.detection_complete and (rospy.Time.now() - start_time) < timeout:
                rate.sleep()
                
            if self.bridge_entrance is None:
                rospy.logwarn('未检测到桥梁或检测超时')
                return 'failed'
            
            # 计算出口坐标（入口坐标y值加上偏移量）
            bridge_exit = PoseStamped()
            bridge_exit.header = self.bridge_entrance.header
            bridge_exit.pose.position.x = self.bridge_entrance.pose.position.x
            bridge_exit.pose.position.y = self.bridge_entrance.pose.position.y + Config.BRIDGE['EXIT_Y_OFFSET']
            bridge_exit.pose.orientation = self.bridge_entrance.pose.orientation
            
            # 将入口和出口坐标保存到userdata中传递给下一个状态
            userdata.bridge_entrance_out = self.bridge_entrance
            userdata.bridge_exit_out = bridge_exit
            
            rospy.loginfo('桥梁检测成功')
            rospy.loginfo('桥入口坐标: x=%.2f, y=%.2f', 
                          self.bridge_entrance.pose.position.x, 
                          self.bridge_entrance.pose.position.y)
            rospy.loginfo('桥出口坐标: x=%.2f, y=%.2f', 
                          bridge_exit.pose.position.x, 
                          bridge_exit.pose.position.y)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('桥梁检测错误: %s', str(e))
            return 'failed'
    
    def bridge_callback(self, msg):
        rospy.loginfo('检测到桥，位置：x=%.2f, y=%.2f', msg.pose.position.x, msg.pose.position.y)
        self.bridge_entrance = msg
        self.detection_complete = True

# 定义状态：导航到桥入口
class NavigateToBridgeEntrance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_entrance_in', 'bridge_exit_in'],
                           output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        
    def execute(self, userdata):
        # 从userdata中获取桥梁入口坐标
        bridge_entrance = userdata.bridge_entrance_in
        
        if bridge_entrance is None:
            rospy.logwarn('缺少桥梁入口坐标信息')
            return 'failed'
        
        rospy.loginfo('开始导航到桥入口...')
        
        try:
            # 导航到桥入口
            rospy.loginfo('导航到桥入口: x=%.2f, y=%.2f', 
                         bridge_entrance.pose.position.x, 
                         bridge_entrance.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_entrance.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_entrance.pose
            
            self.client.send_goal(goal)
            
            # 添加超时
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('导航到桥入口超时')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('导航到桥入口失败')
                return 'failed'
            
            rospy.loginfo('已成功到达桥入口')
            # 传递坐标到下一个状态
            userdata.bridge_entrance_out = bridge_entrance
            userdata.bridge_exit_out = userdata.bridge_exit_in
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('导航到桥入口时出错: %s', str(e))
            return 'failed'

# 定义状态：开桥并导航过桥
class OpenBridgeAndNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_exit_in'])
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.open_bridge_publisher = rospy.Publisher(
            Config.TOPICS['OPEN_BRIDGE'], 
            Bool, 
            queue_size=10
        )
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.bridge_open_wait_time = Config.TIMEOUTS['BRIDGE_OPEN']
        
    def execute(self, userdata):
        # 从userdata中获取桥梁出口坐标
        bridge_exit = userdata.bridge_exit_in
        
        if bridge_exit is None:
            rospy.logwarn('缺少桥梁出口坐标信息')
            return 'failed'
        
        try:
            # 发送开桥指令
            open_bridge_msg = Bool()
            open_bridge_msg.data = True
            self.open_bridge_publisher.publish(open_bridge_msg)
            rospy.loginfo('已发送开桥指令')
            
            # 给桥梁足够的打开时间
            rospy.sleep(self.bridge_open_wait_time)
            
            # 导航到桥出口（过桥）
            rospy.loginfo('开始过桥，导航到桥出口: x=%.2f, y=%.2f', 
                         bridge_exit.pose.position.x, 
                         bridge_exit.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_exit.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_exit.pose
            
            self.client.send_goal(goal)
            
            # 添加超时
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('过桥导航超时')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('过桥失败')
                return 'failed'
            
            rospy.loginfo('成功过桥')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('过桥时出错: %s', str(e))
            return 'failed'

# 定义状态：导航到目标并启动OCR
class NavigateToGoalAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # 获取目标盒子的坐标列表
        self.goals = Config.GOALS
        
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        self.ocr_trigger_publisher = rospy.Publisher(
            Config.TOPICS['OCR_TRIGGER'], 
            Bool, 
            queue_size=10
        )

        # 一旦匹配到目标盒子，OCR节点会发布/cmd_stop
        self.cmd_stop_subscriber = rospy.Subscriber(
            Config.TOPICS['CMD_STOP'], 
            Bool, 
            self.cmd_stop_callback
        )
        self.cmd_stop_received = False  # 标记是否收到停止指令
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        
    def execute(self, userdata):
        rospy.loginfo('执行导航到目标任务...')
        
        try:
            # 遍历目标坐标，依次导航
            for i, goal_position in enumerate(self.goals):
                self.goal.target_pose.pose.position.x = goal_position[0]
                self.goal.target_pose.pose.position.y = goal_position[1]
                self.goal.target_pose.pose.orientation.w = goal_position[2]
                
                rospy.loginfo('[%d/%d] 导航到目标位置: x=%.2f, y=%.2f', 
                              i+1, len(self.goals),
                              goal_position[0], goal_position[1])
                              
                self.client.send_goal(self.goal)
                
                # 添加超时
                if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                    rospy.logwarn('导航到目标位置超时，尝试下一个目标')
                    continue
                    
                result = self.client.get_state()
                
                if result == actionlib.GoalStatus.SUCCEEDED:
                    # 启动OCR处理
                    ocr_trigger_msg = Bool()
                    ocr_trigger_msg.data = True
                    self.ocr_trigger_publisher.publish(ocr_trigger_msg)
                    rospy.loginfo('OCR触发消息已发布，等待处理...')
                    
                    # 等待OCR处理完成
                    wait_start_time = rospy.Time.now()
                    rate = rospy.Rate(2)  # 2Hz检查频率
                    
                    while (rospy.Time.now() - wait_start_time) < self.ocr_timeout:
                        if self.cmd_stop_received:
                            rospy.loginfo('OCR任务完成，停止导航')
                            self.cmd_stop_received = False
                            return 'succeeded'
                        rate.sleep()
                        
                    rospy.loginfo('OCR处理超时，继续导航到下一目标')
                else:
                    rospy.logwarn('导航到目标位置失败，尝试下一个目标')

            rospy.loginfo('所有目标位置导航完成')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('导航到目标时出错: %s', str(e))
            return 'failed'
    
    def cmd_stop_callback(self, msg):
        if msg.data:
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
                               transitions={'initialized':'EXPLORE_FRONTIER',
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
                                          'bridge_exit_in':'bridge_exit',
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
    
    rospy.loginfo('状态机执行完毕，结果：%s', outcome)
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()