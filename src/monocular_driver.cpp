#include "slam/monocular.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<MonocularSlamNode>(); 
    
    // rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------


