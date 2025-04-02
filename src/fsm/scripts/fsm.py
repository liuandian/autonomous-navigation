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
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray

# 可视化
import matplotlib.pyplot as plt
import numpy as np

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
    
    # 禁入区域设置
    RESTRICTED_MAP_BOUNDS = {
        'X_MIN': 0.0,    # 地图X坐标最小值
        'X_MAX': 11.0,   # 地图X坐标最大值
        'Y_MIN': 0.0,    # 地图Y坐标最小值
        'Y_MAX': 22.0,   # 地图Y坐标最大值
    }

    # 探索区域设置
    EXPLORE_MAP_BOUNDS = {
        'X_MIN': 11.0,    # 地图X坐标最小值
        'X_MAX': 22.0,   # 地图X坐标最大值
        'Y_MIN': 2.0,    # 地图Y坐标最小值
        'Y_MAX': 21.0,   # 地图Y坐标最大值
    }
    
    # 掩码操作设置, 如果使用的是my_map_exploration.map，则需要设置USE_EXPLORE_MASK为False
    # 因为my_map_explore地图自带探索区域
    # 如果想直接使用原图，可以一键设置USE_MASKED为True
    MASKED_CONFIG = {
        'USE_MASKED': False,  # 是否使用掩码
        'USE_RESOLUTION': True,  # 是否使用分辨率
        'USE_RESTRICTED_MASK': False,  # 是否使用禁入区域
        'USE_EXPLORE_MASK': False,  # 是否使用探索区域
    }

    # 探索相关设置
    EXPLORE = {
        'FRONTIER_THRESHOLD': 0,     # 前沿点数量阈值，低于此值认为探索完成
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
        
        # 等待地图数据就绪
        while not rospy.is_shutdown():
            with self.map_lock:
                has_map_data = (len(self.costmap_msg.data) > 0)
            
            if not has_map_data:
                rospy.loginfo('等待地图数据...')
                rate.sleep()
                continue
            
            break  # 跳出循环，开始主处理逻辑
        
        # 主处理循环
        rospy.loginfo('地图数据就绪，开始探索...')
        
        while not rospy.is_shutdown():
            # 创建掩码地图，并立即释放锁
            masked_costmap = None
            masked_costmap_updates = None
            frontier_count = 0
            
            # 使用锁保护地图数据的访问，但尽量减少持有锁的时间
            with self.map_lock:
                # 首先，检查地图数据是否有效
                if len(self.costmap_msg.data) == 0:
                    rospy.loginfo('等待有效的地图数据...')
                    rate.sleep()
                    continue
                    
                # 处理地图数据
                masked_costmap = self.process_costmap(self.costmap_msg) if Config.MASKED_CONFIG['USE_MASKED'] else self.costmap_msg
                masked_costmap_updates = self.process_costmap_updates(self.costmap_updates_msg) if Config.MASKED_CONFIG['USE_MASKED'] else self.costmap_updates_msg
                frontier_count = self.frontier_count
            
            # 发布处理过的地图（在锁外）
            self.explore_costmap_publisher.publish(masked_costmap)
            self.explore_costmap_updates_publisher.publish(masked_costmap_updates)
            rospy.loginfo('掩码代价地图已发布')
            
            # 可视化代码也移到锁外
            try:
                # 将可视化部分放在单独的 try-except 中，避免影响主功能
                width = masked_costmap.info.width
                height = masked_costmap.info.height
                data_2d = np.array(masked_costmap.data).reshape(height, width)
                plt.figure(figsize=(8, 6))
                plt.imshow(data_2d, cmap='gray', interpolation='nearest')
                plt.colorbar(label='占用概率')
                plt.title('掩码代价地图')
                plt.savefig('/tmp/masked_costmap.png')
                plt.close()  # 关闭图形，避免内存泄漏
            except Exception as e:
                rospy.logwarn('可视化地图时出错: %s', str(e))
            
            # 检查前沿点数量
            rospy.loginfo('当前前沿点数量: %d (阈值: %d)', frontier_count, self.frontier_threshold)
            
            # 如果前沿点数量小于阈值，认为探索完成
            if frontier_count <= self.frontier_threshold:
                rospy.loginfo('前沿点数量已低于阈值，探索任务完成')
                return 'succeeded'
            
            # 检查是否超时
            if (rospy.Time.now() - explore_start_time).to_sec() > self.max_explore_time:
                rospy.loginfo('探索任务超时，强制完成')
                return 'succeeded'
            
            rate.sleep()

        # 停止探索
        rospy.loginfo('停止前沿探索')
        # 取消订阅器
        self.costmap_subscriber.unregister()
        self.costmap_updates_subscriber.unregister()
        self.frontier_subscriber.unregister()
        # 取消发布器
        self.explore_costmap_publisher.unregister()
        self.explore_costmap_updates_publisher.unregister()
        
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
        
    def process_costmap(self, costmap_msg):
        """处理代价地图，标记要探索的区域为自由空间，其他区域为障碍物"""
        import copy
        # 复制原始代价地图
        masked_costmap = copy.deepcopy(costmap_msg)
        
        # 更新时间戳和帧 ID
        masked_costmap.header.stamp = rospy.Time.now()
        if masked_costmap.header.frame_id == "":
            masked_costmap.header.frame_id = "map"
        
        # 获取地图信息
        width = masked_costmap.info.width
        height = masked_costmap.info.height
        resolution = masked_costmap.info.resolution if Config.MASKED_CONFIG['USE_RESOLUTION'] else 1.0
        origin_x = masked_costmap.info.origin.position.x
        origin_y = masked_costmap.info.origin.position.y
        
        # 计算禁入区域的边界
        restricted_area_x_min = int((Config.RESTRICTED_MAP_BOUNDS['X_MIN'] ) / resolution)
        restricted_area_x_max = int((Config.RESTRICTED_MAP_BOUNDS['X_MAX'] ) / resolution)
        restricted_area_y_min = int((Config.RESTRICTED_MAP_BOUNDS['Y_MIN'] ) / resolution)
        restricted_area_y_max = int((Config.RESTRICTED_MAP_BOUNDS['Y_MAX'] ) / resolution)
        
        # 限制索引在地图范围内
        restricted_area_x_min = max(0, min(width-1, restricted_area_x_min))
        restricted_area_x_max = max(0, min(width-1, restricted_area_x_max))
        restricted_area_y_min = max(0, min(height-1, restricted_area_y_min))
        restricted_area_y_max = max(0, min(height-1, restricted_area_y_max))

        # 计算探索区域的边界
        explore_area_x_min = int((Config.EXPLORE_MAP_BOUNDS['X_MIN'] ) / resolution)
        explore_area_x_max = int((Config.EXPLORE_MAP_BOUNDS['X_MAX'] ) / resolution)
        explore_area_y_min = int((Config.EXPLORE_MAP_BOUNDS['Y_MIN'] ) / resolution)
        explore_area_y_max = int((Config.EXPLORE_MAP_BOUNDS['Y_MAX'] ) / resolution)

        # 限制索引在地图范围内
        explore_area_x_min = max(0, min(width-1, explore_area_x_min))
        explore_area_x_max = max(0, min(width-1, explore_area_x_max))
        explore_area_y_min = max(0, min(height-1, explore_area_y_min))
        explore_area_y_max = max(0, min(height-1, explore_area_y_max))
        
        rospy.loginfo('禁入区域边界: (%d, %d) - (%d, %d)', 
                      restricted_area_x_min, restricted_area_y_min,
                      restricted_area_x_max, restricted_area_y_max)
        rospy.loginfo('探索区域边界: (%d, %d) - (%d, %d)', 
                      explore_area_x_min, explore_area_y_min,
                      explore_area_x_max, explore_area_y_max)
        
        # 将数据转换为可变列表
        data_list = list(masked_costmap.data)
        
        # 标记禁入区域为障碍物，探索区域为未知，其余不变
        modified_cells = 0
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if idx < len(data_list):
                    if (restricted_area_x_min <= x <= restricted_area_x_max and
                        restricted_area_y_min <= y <= restricted_area_y_max and
                        Config.MASKED_CONFIG['USE_RESTRICTED_MASK']):
                        # 在禁入区域内：标记为障碍物
                        data_list[idx] = 100
                        modified_cells += 1
                    elif (explore_area_x_min <= x <= explore_area_x_max and 
                          explore_area_y_min <= y <= explore_area_y_max and 
                          Config.MASKED_CONFIG['USE_EXPLORE_MASK']):
                        # 在探索区域内：标记为未知
                        data_list[idx] = -1
                        modified_cells += 1
                    else:
                        # 其他区域：保持原值
                        pass
                        
        
        # 转换回元组
        masked_costmap.data = tuple(data_list)
        
        rospy.loginfo('掩码代价地图处理完成: 修改了 %d 个单元格', modified_cells)
        
        return masked_costmap

    def process_costmap_updates(self, update_msg):
        """处理代价地图更新，设置禁入区域为障碍物，探索区域为未知"""
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
        resolution = self.last_complete_map.info.resolution if Config.MASKED_CONFIG['USE_RESOLUTION'] else 1.0
        origin_x = self.last_complete_map.info.origin.position.x
        origin_y = self.last_complete_map.info.origin.position.y
        
        # 计算更新区域在实际世界中的绝对坐标
        update_origin_x = origin_x + x_start * resolution
        update_origin_y = origin_y + y_start * resolution
        
        # 获取禁入区域的世界坐标边界
        restricted_x_min_world = Config.RESTRICTED_MAP_BOUNDS['X_MIN']
        restricted_x_max_world = Config.RESTRICTED_MAP_BOUNDS['X_MAX']
        restricted_y_min_world = Config.RESTRICTED_MAP_BOUNDS['Y_MIN']
        restricted_y_max_world = Config.RESTRICTED_MAP_BOUNDS['Y_MAX']
        
        # 获取探索区域的世界坐标边界
        explore_x_min_world = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        explore_x_max_world = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        explore_y_min_world = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        explore_y_max_world = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        
        # 将世界坐标转换为更新区域内的局部坐标
        # 首先计算这些边界在全局地图中的单元格坐标
        restricted_x_min_global = int((restricted_x_min_world - origin_x) / resolution)
        restricted_x_max_global = int((restricted_x_max_world - origin_x) / resolution)
        restricted_y_min_global = int((restricted_y_min_world - origin_y) / resolution)
        restricted_y_max_global = int((restricted_y_max_world - origin_y) / resolution)
        
        explore_x_min_global = int((explore_x_min_world - origin_x) / resolution)
        explore_x_max_global = int((explore_x_max_world - origin_x) / resolution)
        explore_y_min_global = int((explore_y_min_world - origin_y) / resolution)
        explore_y_max_global = int((explore_y_max_world - origin_y) / resolution)
        
        # 然后将全局坐标转换为更新区域内的局部坐标
        restricted_x_min_local = restricted_x_min_global - x_start
        restricted_x_max_local = restricted_x_max_global - x_start
        restricted_y_min_local = restricted_y_min_global - y_start
        restricted_y_max_local = restricted_y_max_global - y_start
        
        explore_x_min_local = explore_x_min_global - x_start
        explore_x_max_local = explore_x_max_global - x_start
        explore_y_min_local = explore_y_min_global - y_start
        explore_y_max_local = explore_y_max_global - y_start
        
        # 限制局部坐标在更新区域范围内
        restricted_x_min_local = max(0, min(width-1, restricted_x_min_local))
        restricted_x_max_local = max(0, min(width-1, restricted_x_max_local))
        restricted_y_min_local = max(0, min(height-1, restricted_y_min_local))
        restricted_y_max_local = max(0, min(height-1, restricted_y_max_local))
        
        explore_x_min_local = max(0, min(width-1, explore_x_min_local))
        explore_x_max_local = max(0, min(width-1, explore_x_max_local))
        explore_y_min_local = max(0, min(height-1, explore_y_min_local))
        explore_y_max_local = max(0, min(height-1, explore_y_max_local))
        
        rospy.loginfo('代价地图更新: 局部禁入区域边界: (%d,%d)-(%d,%d)', 
                    restricted_x_min_local, restricted_y_min_local,
                    restricted_x_max_local, restricted_y_max_local)
        rospy.loginfo('代价地图更新: 局部探索区域边界: (%d,%d)-(%d,%d)', 
                    explore_x_min_local, explore_y_min_local,
                    explore_x_max_local, explore_y_max_local)

        # 标记区域 - 将数据转换为列表以便修改
        data_list = list(masked_update.data)
        modified_cells = 0
        
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if idx < len(data_list):
                    if (restricted_x_min_local <= x <= restricted_x_max_local and 
                        restricted_y_min_local <= y <= restricted_y_max_local and
                        Config.MASKED_CONFIG['USE_RESTRICTED_MASK']):
                        # 在禁入区域内：标记为障碍物
                        data_list[idx] = 100
                        modified_cells += 1
                    elif (explore_x_min_local <= x <= explore_x_max_local and 
                        explore_y_min_local <= y <= explore_y_max_local and
                        Config.MASKED_CONFIG['USE_EXPLORE_MASK']):
                        # 在探索区域内：标记为未知
                        data_list[idx] = -1
                        modified_cells += 1
                    else:
                        # 其他区域：保持原值
                        pass
        
        # 转换回元组并更新消息
        masked_update.data = tuple(data_list)
        
        rospy.loginfo('更新掩码已处理: 修改了 %d 个单元格', modified_cells)
        
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
            
            while (rospy.Time.now() - start_time).to_sec() < self.detection_timeout:
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
        while len(msg.poses) == 0:
            continue
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
            while not self.detection_complete and (rospy.Time.now() - start_time).to_sec() < timeout:
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
                    
                    while (rospy.Time.now() - wait_start_time).to_sec() < self.ocr_timeout:
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