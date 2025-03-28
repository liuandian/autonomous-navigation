#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class MaskedCostmap:
    def __init__(self):
        rospy.init_node('masked_costmap_node')
        
        # 定义感兴趣区域范围 [x_min:x_max, y_min:y_max]
        self.x_min = rospy.get_param('~x_min', 20)
        self.x_max = rospy.get_param('~x_max', 30)
        self.y_min = rospy.get_param('~y_min', 20)
        self.y_max = rospy.get_param('~y_max', 30)
        
        # 订阅原始地图
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # 发布掩码代价地图
        self.costmap_pub = rospy.Publisher('/masked_costmap', OccupancyGrid, queue_size=1, latch=True)
        
    def map_callback(self, map_msg):
        # 复制原始地图消息
        masked_map = OccupancyGrid()
        masked_map.header = map_msg.header
        masked_map.info = map_msg.info
        
        # 获取地图尺寸
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        
        # 原始地图数据转为numpy数组
        data = np.array(map_msg.data).reshape(height, width)
        
        # 计算栅格坐标
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # 将米转换为栅格单元
        x_min_cell = int((self.x_min - origin_x) / resolution)
        x_max_cell = int((self.x_max - origin_x) / resolution)
        y_min_cell = int((self.y_min - origin_y) / resolution)
        y_max_cell = int((self.y_max - origin_y) / resolution)
        
        # 确保在地图范围内
        x_min_cell = max(0, min(x_min_cell, width-1))
        x_max_cell = max(0, min(x_max_cell, width-1))
        y_min_cell = max(0, min(y_min_cell, height-1))
        y_max_cell = max(0, min(y_max_cell, height-1))
        
        # 创建掩码：目标区域外的所有区域标记为占用
        masked_data = np.ones_like(data) * 100  # 将所有区域标记为占用
        masked_data[y_min_cell:y_max_cell, x_min_cell:x_max_cell] = data[y_min_cell:y_max_cell, x_min_cell:x_max_cell]
        
        # 转回一维数组并发布
        masked_map.data = masked_data.flatten().astype(int).tolist()
        self.costmap_pub.publish(masked_map)
        rospy.loginfo("已发布掩码代价地图，只保留区域 [%d:%d, %d:%d]", self.x_min, self.x_max, self.y_min, self.y_max)

if __name__ == '__main__':
    try:
        masked_costmap = MaskedCostmap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass