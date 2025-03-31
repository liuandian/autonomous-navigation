# ME5413 Final Project Group12
## 项目简介

这是ME5413课程的最终项目，实现了具有自主导航、探索、物体检测和OCR功能的机器人系统。此系统基于ROS（机器人操作系统）开发，采用Jackal机器人平台，能够在虚拟环境中完成复杂的导航任务。

## 系统要求

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- Python 2.7/3.x
- 库依赖:
  - OpenCV
  - pytesseract
  - scikit-learn
  - numpy
  - smach

## 项目结构

项目主要包含以下模块:

- **fsm**: 有限状态机模块，协调各个任务的执行
- **ocr**: 光学字符识别模块，用于识别环境中的数字信息
- **box_detection**: 盒子检测模块，基于点云数据检测并可视化环境中的盒子
- **frontier_explore**: 前沿探索模块，实现自主探索未知区域
- **navigation**: 导航模块，负责机器人的路径规划和避障
- **SLAM**: 使用FAST-LIO进行同步定位与地图构建

## 安装步骤

1. 创建ROS工作空间:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. 克隆项目:
```bash
git clone https://github.com/CANLAN-SC/ME5413_Final_Project_Group12.git
```

3. 安装依赖:
```bash
sudo apt-get update
sudo apt-get install python-opencv python-numpy ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base
sudo apt-get install tesseract-ocr libtesseract-dev
pip install pytesseract scikit-learn
```

4. 编译工作空间:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行方法

### 1. 启动所有节点

```bash
roslaunch fsm final.launch
```
应看到终端重复
```bash
[DEBUG] [1743423288.122009188, 605.893000000]: Getting status over the wire.
```
此时仿真环境、rviz、视觉识别、雷达检测、导航、SLAM节点均启动且可以键盘控制仿真小车

> **注意:** 如果遇到 `/usr/bin/env: 'python\r': No such file or directory` 错误，请安装并使用dos2unix修复:
> ```bash
> sudo apt-get install dos2unix
> dos2unix src/ocr/scripts/before_bridge_ocr.py
> dos2unix src/ocr/scripts/after_bridge_ocr.py
> ```

### 2. 加入状态机

新开终端
```bash
cd ME5413_Final_Project_Group12
source devel/setup.bash
python src/fsm/scripts/fsm.py
```

应看到:
```bash
[INFO] [1743423111.980278, 0.000000]: State machine starting in initial state 'INITIALIZE' with userdata: 
        []
[INFO] [1743423111.982093, 0.000000]: Initializing system...
[INFO] [1743423111.983973, 0.000000]: State machine transitioning 'INITIALIZE':'initialized'-->'NAVIGATE_TO_GOAL'
[INFO] [1743423111.985405, 0.000000]: 执行任务三...
[INFO] [1743423111.987305, 0.000000]: State machine transitioning 'NAVIGATE_TO_GOAL':'succeeded'-->'TASK_ONE'
[INFO] [1743423111.988715, 0.000000]: 执行任务一...
[INFO] [1743423111.990916, 0.000000]: 触发消息已发布
[INFO] [1743423127.133987, 562.406000]: State machine transitioning 'TASK_ONE':'succeeded'-->'TASK_TWO'
[INFO] [1743423127.136749, 562.406000]: 执行任务二...
[INFO] [1743423127.139615, 562.408000]: State machine transitioning 'TASK_TWO':'succeeded'-->'TASK_THREE'
[INFO] [1743423127.142591, 562.411000]: 执行任务三...
[INFO] [1743423127.144614, 562.412000]: State machine terminating 'TASK_THREE':'succeeded':'mission_completed'
[INFO] [1743423127.147272, 562.414000]: 状态机执行完毕，结果：mission_completed
```

## 功能说明

- **状态机控制**: 协调导航、探索、检测等任务的执行顺序
- **自主探索**: 使用frontier_explore探索未知区域
- **盒子检测**: 使用DBSCAN聚类算法检测环境中的盒子
- **OCR识别**: 识别环境中的数字，用于桥梁解锁
- **SLAM**: 使用FAST-LIO进行环境建图和定位
- **自主导航**: 使用move_base实现机器人的自主导航

## 系统架构

系统由多个模块组成，主要包括:

1. **主启动文件**：final.launch
2. **传感处理**：fast_lio进行激光SLAM
3. **目标检测**：box_detection检测盒子，ocr进行字符识别
4. **路径规划**：move_base和TEB规划器
5. **前沿探索**：frontier_explore实现未知区域探索
6. **任务协调**：fsm状态机协调各任务执行

```mermaid
graph LR
    %% 主启动文件
    final[final.launch] 
    
    %% 包含的其他launch文件
    world[world.launch]
    manual[manual.launch]
    ocr_launch[ocr.launch]
    box_detection_launch[box_detection.launch]
    frontier_explore_launch[frontier_explore.launch]
    move_base_launch[move_base.launch]
    fast_lio_launch[fast_lio.launch]
    masked_costmap_launch[masked_costmap.launch]
    frontier_filter_launch[frontier_filter.launch]
    
    %% 节点
    gazebo[gazebo_ros]
    jackal[spawn_jackal]
    teleop[teleop_twist_keyboard]
    rviz[rviz]
    box_detection_node[box_detection]
    ocr_node[ocr_node/before_bridge_ocr.py]
    explore_node[explore_lite/explore]
    masked_costmap_node[masked_costmap.py]
    frontier_filter_node[frontier_filter.py]
    move_base_node[move_base]
    fast_lio_node[fastlio_mapping]
    
    %% FSM状态机节点和状态
    fsm_node[task_coordinator]
    init_state[Initialize]
    nav_to_explore_state[NavigateToExplorationArea]
    explore_state[ExploreFrontier]
    detect_bridge_state[DetectBridge]
    nav_to_goal_state[NavigateToGoal]
    
    %% 话题
    mid_points[/mid/points\]
    detected_boxes[/detected_boxes\]
    bounding_boxes[/bounding_boxes\]
    front_image[/front/image_raw\]
    ocr_trigger[/ocr_trigger\]
    cmd_open_bridge[/cmd_open_bridge\]
    recognized_digit[/recognized_digit\]
    mode_digit[/mode_digit\]
    map_topic[/map\]
    masked_costmap[/masked_costmap\]
    explore_frontiers[/explore/frontiers\]
    filtered_frontiers[/filtered_frontiers\]
    move_base_goal[/move_base/goal\]
    move_base_result[/move_base/result\]
    move_base_cancel[/move_base/cancel\]
    odometry_filtered[/odometry/filtered\]
    cmd_vel[/cmd_vel\]
    global_costmap[/move_base/global_costmap/costmap\]
    
    %% TF坐标系
    tf_base_link((base_link))
    tf_map((map))
    tf_odom((odom))
    tf_lidar((lidar_link))
    
    %% Launch文件层次结构
    final --> world
    final --> manual
    final --> ocr_launch
    final --> box_detection_launch
    final --> frontier_explore_launch
    final --> move_base_launch
    final --> fast_lio_launch
    
    %% launch文件启动的节点
    world --> gazebo
    world --> jackal
    manual --> teleop
    manual --> rviz
    ocr_launch --> ocr_node
    box_detection_launch --> box_detection_node
    frontier_explore_launch --> explore_node
    frontier_explore_launch --> masked_costmap_launch
    frontier_explore_launch --> frontier_filter_launch
    masked_costmap_launch --> masked_costmap_node
    frontier_filter_launch --> frontier_filter_node
    move_base_launch --> move_base_node
    fast_lio_launch --> fast_lio_node
    fast_lio_launch --> teleop
    
    %% FSM内部状态关系
    fsm_node --> init_state
    init_state -->|initialized| nav_to_explore_state
    init_state -->|failed| mission_failed[任务失败]
    nav_to_explore_state -->|succeeded| explore_state
    nav_to_explore_state -->|failed/preempted| mission_failed
    explore_state -->|succeeded| detect_bridge_state
    explore_state -->|failed| mission_failed
    detect_bridge_state -->|succeeded| nav_to_goal_state
    detect_bridge_state -->|failed| mission_failed
    nav_to_goal_state -->|succeeded| mission_completed[任务完成]
    nav_to_goal_state -->|failed| mission_failed
    
    %% 话题订阅与发布关系
    box_detection_node -- 订阅 --> mid_points
    box_detection_node -- 发布 --> detected_boxes
    box_detection_node -- 发布 --> bounding_boxes
    
    ocr_node -- 订阅 --> front_image
    ocr_node -- 订阅 --> ocr_trigger
    ocr_node -- 订阅 --> cmd_open_bridge
    ocr_node -- 发布 --> recognized_digit
    ocr_node -- 发布 --> mode_digit
    
    masked_costmap_node -- 订阅 --> map_topic
    masked_costmap_node -- 发布 --> masked_costmap
    
    frontier_filter_node -- 订阅 --> explore_frontiers
    frontier_filter_node -- 订阅 --> move_base_goal
    frontier_filter_node -- 发布 --> filtered_frontiers
    frontier_filter_node -- 发布 --> move_base_cancel
    
    explore_node -- 订阅 --> global_costmap
    explore_node -- 发布 --> explore_frontiers
    explore_node -- 发布 --> move_base_goal
    
    move_base_node -- 订阅 --> odometry_filtered
    move_base_node -- 订阅 --> map_topic
    move_base_node -- 发布 --> cmd_vel
    move_base_node -- 发布 --> global_costmap
    move_base_node -- 发布 --> move_base_result
    
    teleop -- 发布 --> cmd_vel
    
    fast_lio_node -- 订阅 --> mid_points
    fast_lio_node -- 发布 --> map_topic
    fast_lio_node -- 发布 --> odometry_filtered
    
    %% FSM与其他节点的交互
    explore_state -- 发布 --> ocr_trigger
    nav_to_explore_state -- 发布 --> move_base_goal
    nav_to_explore_state -- 订阅 --> move_base_result
    nav_to_goal_state -- 发布 --> move_base_goal
    nav_to_goal_state -- 订阅 --> move_base_result
    detect_bridge_state -- 订阅 --> detected_boxes
    detect_bridge_state -- 发布 --> cmd_open_bridge
    
    %% TF坐标转换关系
    jackal -- 发布 --> tf_base_link
    jackal -- 发布 --> tf_lidar
    fast_lio_node -- 发布 --> tf_map
    fast_lio_node -- 发布 --> tf_odom
    
    tf_map -- 转换 --> tf_odom
    tf_odom -- 转换 --> tf_base_link
    tf_base_link -- 转换 --> tf_lidar
    
    %% 样式设置
    classDef launch fill:#f9d,stroke:#333,stroke-width:2px;
    classDef node fill:#bbf,stroke:#333,stroke-width:2px;
    classDef state fill:#aaf,stroke:#333,stroke-width:2px;
    classDef topic fill:#ddd,stroke:#333,stroke-width:1px;
    classDef tf fill:#ffd700,stroke:#333,stroke-width:2px;
    classDef outcome fill:#d9f,stroke:#333,stroke-width:1px;
    
    class final,world,manual,ocr_launch,box_detection_launch,frontier_explore_launch,move_base_launch,fast_lio_launch,masked_costmap_launch,frontier_filter_launch launch;
    class gazebo,jackal,teleop,rviz,box_detection_node,ocr_node,explore_node,masked_costmap_node,frontier_filter_node,move_base_node,fast_lio_node,fsm_node node;
    class init_state,nav_to_explore_state,explore_state,detect_bridge_state,nav_to_goal_state state;
    class mid_points,detected_boxes,bounding_boxes,front_image,ocr_trigger,cmd_open_bridge,recognized_digit,mode_digit,map_topic,masked_costmap,explore_frontiers,filtered_frontiers,move_base_goal,move_base_result,move_base_cancel,odometry_filtered,cmd_vel,global_costmap topic;
    class tf_base_link,tf_map,tf_odom,tf_lidar tf;
    class mission_completed,mission_failed outcome;
```


## 贡献者

- Group 12 成员

## 许可证

LICENSE文件中详细说明