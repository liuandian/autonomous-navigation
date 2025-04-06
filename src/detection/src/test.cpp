#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// #include <ros/console.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
#include <chrono>
#include <thread>

class BoxCountNode {
public:
    BoxCountNode() {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 订阅话题
        lidar_sub = nh.subscribe("/mid/points", 10, &BoxCountNode::lidarCallback, this);
        image_sub = nh.subscribe("/front/image_raw", 10, &BoxCountNode::imageCallback, this);

        // 初始化时间
        start_time_lidar = ros::Time::now();
        start_time_image = ros::Time::now();
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 记录当前时间
        ros::Time end_time_lidar = ros::Time::now();
        double elapsed_time = (end_time_lidar - start_time_lidar).toSec();
        ROS_INFO("Lidar Time elapsed: %.2f seconds", elapsed_time);

        // 更新开始时间
        start_time_lidar = end_time_lidar;

        // 在这里处理点云数据（如果需要）
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::fromROSMsg(*msg, *cloud);
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // 记录当前时间
        ros::Time end_time_image = ros::Time::now();
        double elapsed_time = (end_time_image - start_time_image).toSec();
        ROS_INFO("Camera Time elapsed: %.2f seconds", elapsed_time);

        // 更新开始时间
        start_time_image = end_time_image;

        // 在这里处理图像数据（如果需要）
    }

private:
    ros::Subscriber lidar_sub;
    ros::Subscriber image_sub;

    ros::Time start_time_lidar;
    ros::Time start_time_image;
};

int main(int argc, char** argv) {
     // 创建节点对象
     
    // 初始化 ROS 节点
    ros::init(argc, argv, "ocr_lidar_box_center_node");
    BoxCountNode node;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
   



    return 0;
}