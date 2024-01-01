/*

TODO write something here i.e purpose of this node and what it does
* pass

NOTES
* pass

*/

#include "ros2_orb_slam3/robot_slam.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono-orb-slam3")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    this->declare_parameter("settings_file_name_arg", "file_not_set"); // Which settings file? i.e. EuRoC, LSU-iCORE mono etc  


    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* Build the full path
    settingsFilePath = settingsFilePath.append(param4.as_string());
    settingsFilePath = settingsFilePath.append(".yaml"); // ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", agentName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    

    subexperimentconfigName = "/" + agentName + "/experiment_settings"; // subscription topic name
    pubconfigackName = "/" + agentName + "/exp_settings_ack"; // publish topic name
    
    subMatimgName = "/" + agentName + "/matimg_msg"; // subscription topic name

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&RobotMonocularSLAM::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subMatimg_subscription_= this->create_subscription<matimg_custom_msg_interface::msg::MatImg>(subMatimgName, 1, std::bind(&RobotMonocularSLAM::matImg_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Saving trajectory and any further clean are done here
    pAgent->Shutdown();

    // // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // TODO take name of the 
    
    // Error if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    // TODO populate these from 
    
    sensorType = ORB_SLAM3::System::MONOCULAR; // TODO make part of launch file
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    // pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, experimentConfig, agentName, enableOpenCVWindow, enablePangolinWindow);
    
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    
    //std::cout<<"Received configuration: "<<experimentConfig<<std::endl;
    RCLCPP_INFO(this->get_logger(), "experimentConfig received: %s", experimentConfig.c_str());
    
    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    // initializeVSLAM(experimentConfig);

}

//* Callback to process image and semantic matrix from python node
void MonocularMode::Img_callback(const matimg_custom_msg_interface::msg::MatImg& msg)
{
    // std::cout<<"in matImg_callback()"<<std::endl;
    
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg.img); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        cv::imshow("test_window", cv_ptr->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    //* Convert timestamp into C++ double
    // double timestamp = msg.timestamp;

    //* TODO
    //Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp); //* Entry point to ORB_SLAM3 pipeline (overloaded TrackMonocular)
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}


