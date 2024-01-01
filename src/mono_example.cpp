/*
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
* Modified to accept semantix matrix and image message from py_nn node
* Modified by: Azmyin Md. Kamal
* Version: 1.0
* Date: 12/03/23
* Compatible for ROS2
*/

//* Import all necessary modules
#include "orb_slam3_ros2/robot_slam.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<RobotMonocularSLAM>(); 
    
    // Set the desired update rate (e.g., 10 Hz)
    // rclcpp::Rate rate(20);

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------


