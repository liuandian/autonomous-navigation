# ME5413_Final_Project

NUS ME5413 Autonomous Mobile Robotics Final Project
> 
![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)

![cover_image](src/me5413_world/media/gz_world.png)


## Installation


```bash
# Clone your own fork of this repo (assuming home here `~/`)
cd
git https://github.com/GoFuuu/ME5413_Final_Project.git


cd ME5413_Final_Project

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
# Source 
source devel/setup.bash
```

To properly load the gazebo world, you will need to have the necessary model files in the `~/.gazebo/models/` directory.

There are two sources of models needed:

* [Gazebo official models](https://github.com/osrf/gazebo_models)
  
  ```bash
  # Create the destination directory
  cd
  mkdir -p .gazebo/models

  # Clone the official gazebo models repo (assuming home here `~/`)
  git clone https://github.com/osrf/gazebo_models.git

  # Copy the models into the `~/.gazebo/models` directory
  cp -r ~/gazebo_models/* ~/.gazebo/models
  ```

* [Our customized models](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/tree/main/src/me5413_world/models)

  ```bash
  # Copy the customized models into the `~/.gazebo/models` directory
  cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models
  ```

## Usage
```bash

#Build the workspace
cd ME5413_Final_Project
catkin_make

#source 
source devel/setup.bash

#State machine
rosrun stateMachine state_machine.py
#Nav
roslaunch stateMachine final.launch


#Box detection
rosrun detection boxes_detection.py
#Bridge detection
rosrun bridge_detector bridge_detector



```