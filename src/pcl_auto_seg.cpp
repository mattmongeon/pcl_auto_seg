#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_auto_seg");
    ros::NodeHandle nh;

    std::cout << "I'm alive" << std::endl;

    ros::spin();

    return 0;
}

