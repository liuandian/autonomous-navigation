roslaunch fsm final.launch

rviz rviz

rostopic pub /bridge_detection_trigger std_msgs/Bool "data: true"



rosrun  bridge_detector bridge_detector_test.py



roslaunch bridge_detector bridge_detector.launch



rosrun fsm bridge_detector.py 
rostopic pub -r 1 /bridge_detection_trigger std_msgs/Bool "data: true"



rostopic echo -n 1 /move_base/global_costmap/costmap/info


rostopic echo /move_base_simple/goal 
rostopic echo -n 1 /amcl_pose

