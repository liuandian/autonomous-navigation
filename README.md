

1. 安装依赖:
```bash
sudo apt-get update
sudo apt-get install python-opencv python-numpy ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base
sudo apt-get install tesseract-ocr libtesseract-dev
sudo apt-get install ros-noetic-explore-lite
sudo apt-get install ros-noetic-teb-local-planner
pip install pytesseract scikit-learn
```

1. 编译工作空间:
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
