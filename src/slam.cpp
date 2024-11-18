#include "slam/slam.hpp"

bool readYAMLFile(std::string &yamlPath, YAML::Node &output){
	if(std::filesystem::exists(yamlPath)){
		if(!std::filesystem::is_directory(yamlPath)){
			try{
				output = YAML::LoadFile(yamlPath);
			}
			catch(const std::exception& err){
				std::cout<<err.what()<<std::endl;
			}
			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}

Frame::Frame(std::shared_ptr<cv::Mat> image, long &timestamp){
	mpImage = image;
	mpTimestamp = timestamp;
}

Slam::Slam(rclcpp::Logger logger): mpLogger(logger){
	RCLCPP_INFO(mpLogger, "Creating SLAM Base Object");
}
