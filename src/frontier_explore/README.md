# frontier_explore

## 调用的动作

**move_base** (move_base_msgs/MoveBaseAction)  
move_base actionlib API 用于发布导航目标。详情请参见 move_base#Action API。这要求 move_base 节点与 explore_lite 在同一命名空间中，如果不是这样，您可能需要重新映射该节点。

## 订阅的话题

**costmap** (nav_msgs/OccupancyGrid)  
用于探索规划的地图。可以是来自 move_base 的代价地图或由 SLAM 创建的地图（见上文）。占用栅格必须正确标记未知空间，地图构建算法通常默认会跟踪未知空间。如果您想使用 move_base 提供的代价地图，需要通过设置 track_unknown_space: true 来启用未知空间跟踪。

**costmap_updates** (map_msgs/OccupancyGridUpdate)  
代价地图的增量更新。如果地图源始终发布完整更新（即不提供此话题），则不必订阅。

## 发布的话题

**~frontiers** (visualization_msgs/MarkerArray)  
探索算法考虑的前沿可视化。每个前沿都通过蓝色的前沿点和一个小球体来可视化，小球体表示前沿的成本（成本更高的前沿会有更小的球体）。

## 参数

**~robot_base_frame** (string, 默认值: base_link)  
机器人基座坐标系的名称。用于确定机器人在地图上的位置。必需参数。

**~costmap_topic** (string, 默认值: costmap)  
指定源 nav_msgs/OccupancyGrid 的话题。必需参数。

**~costmap_updates_topic** (string, 默认值: costmap_updates)  
指定源 map_msgs/OccupancyGridUpdate 的话题。如果地图源始终发布完整更新（即不提供此话题），则不必设置。

**~visualize** (bool, 默认值: false)  
指定是否发布前沿可视化信息。

**~planner_frequency** (double, 默认值: 1.0)  
计算新前沿和重新考虑目标的频率，单位为赫兹。

**~progress_timeout** (double, 默认值: 30.0)  
时间，单位为秒。当机器人在 progress_timeout 时间内没有取得任何进展时，当前目标将被放弃。

**~potential_scale** (double, 默认值: 1e-3)  
用于前沿权重计算。这个乘法参数影响前沿权重的潜力组件（到前沿的距离）。

**~orientation_scale** (double, 默认值: 0)  
用于前沿权重计算。这个乘法参数影响前沿权重的方向组件。该参数目前不起作用，仅为向前兼容性而提供。

**~gain_scale** (double, 默认值: 1.0)  
用于前沿权重计算。这个乘法参数影响前沿权重的增益组件（前沿大小）。

**~transform_tolerance** (double, 默认值: 0.3)  
变换机器人姿态时使用的容差。

**~min_frontier_size** (double, 默认值: 0.5)  
考虑前沿作为探索目标的最小前沿尺寸，单位为米。

## 所需的 tf 变换

**global_frame → robot_base_frame**  
此变换通常由地图构建算法提供。这些坐标系通常称为 map 和 base_link。有关调整 robot_base_frame 名称，请参见相应参数。您不需要设置 global_frame。global_frame 的名称将自动从 costmap_topic 中获取。