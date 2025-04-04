#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint {
    int index;
    double x;
    double y;
    double yaw;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    // 定义导航目标点
    std::vector<Waypoint> waypoints = {
        {0, 5, 6, -3.14}
        // {1, 8, 14,1.57}
        // {2, 4.0, 7.519,1.57},
        // {3, 1.308, -7.952,1.57},
        // {4,-3.203, -8.422,1.57}
    };

    MoveBaseClient ac("move_base", true);

    // 等待MoveBaseAction服务器启动
    ROS_INFO("Waiting for the move_base action server to start...");
    ac.waitForServer();

    ROS_INFO("move_base action server started!");

    // 连续发布导航目标点
    for (const auto& waypoint : waypoints) {
        // 创建MoveBaseGoal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoint.x;
        goal.target_pose.pose.position.y = waypoint.y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint.yaw);

        ROS_INFO("Sending goal for waypoint %d", waypoint.index);

        // 发布目标点
        ac.sendGoal(goal);

        // 等待导航到达目标点
        ac.waitForResult(ros::Duration(200.0));

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Waypoint %d reached successfully!", waypoint.index);
        } else {
            ROS_WARN("Failed to reach waypoint %d", waypoint.index);
        }

        // 暂停一段时间，以便机器人到达目标点
        // ros::Duration(1.0).sleep();
    }

    return 0;
}