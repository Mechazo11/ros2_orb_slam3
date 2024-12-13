#include "slam/orbslam3/monocular.hpp"

#include <opencv2/core/core.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

using namespace std;

using std::placeholders::_1;

MonoORBSLAM3::MonoORBSLAM3(rclcpp::Logger logger) : Slam(logger){
	RCLCPP_INFO(mpLogger, "Creating ORBSLAM3 Object");	
}

void MonoORBSLAM3::InitialiseSlam(
		std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){
	RCLCPP_INFO(mpLogger, "Initialising ORBSLAM3 Object");
	std::string configFilePath = request->config_file_path;		
	std::string vocabFilePath, settingsFilePath;
	ORB_SLAM3::System::eSensor cameraType;
	YAML::Node slamConfig;
	if(readYAMLFile(configFilePath, slamConfig)){
		RCLCPP_INFO(mpLogger, "successfully read slam config");
		if(slamConfig["Slam.VocabFilePath"].IsDefined()){
			vocabFilePath = slamConfig["Slam.VocabFilePath"].as<std::string>();	
			RCLCPP_INFO(mpLogger, "got vocab file path: %s", vocabFilePath);
			// mpVocabFilePath = vocabFilePath;
		}
		else{
			response->success = false;
			response->message = "Vocab file path not found";
		}
		if(slamConfig["Slam.SettingsFilePath"].IsDefined()){
			settingsFilePath = slamConfig["Slam.SettingsFilePath"].as<std::string>();
			RCLCPP_INFO(mpLogger, "got settings file path: %s", settingsFilePath);
			// mpSettingsFilePath = settingsFilePath;
		}
		else{
			response->success = false;
			response->message = "Settings file path not found";
		}
		if((request->camera_type).compare("monocular") == 0){
			cameraType = ORB_SLAM3::System::eSensor::MONOCULAR;	
		}
		else{
			response->success = false;
			response->message = "Received unsupported camera type";
		}
		mpORBSlam3 = std::make_unique<ORB_SLAM3::System>(vocabFilePath, settingsFilePath, cameraType);
	}
}

void MonoORBSLAM3::TrackMonocular(Frame &frame, Sophus::SE3f &tcw){
	tcw = mpORBSlam3->TrackMonocular(frame.getImage(), frame.getTimestampSec());
}


void MonoORBSLAM3::SetFrameMapPointUpdateCallback(std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> callback){
	mpORBSlam3->SetFrameMapPointUpdateCallback(callback);
}
