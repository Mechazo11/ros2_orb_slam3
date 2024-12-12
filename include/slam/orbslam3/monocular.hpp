#ifndef MONOCULAR_SLAM_NODE_HPP
#define MONOCULAR_SLAM_NODE_HPP


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include "slam/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <cv_bridge/cv_bridge.h>
// ORB_SLAM3 related includes
#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "MapPoint.h"

class MonoORBSLAM3 : public Slam{
	public:
		MonoORBSLAM3(rclcpp::Logger logger);
		~MonoORBSLAM3(){
			if(mpORBSlam3){
				mpORBSlam3->Shutdown();
			}
			RCLCPP_INFO(mpLogger, "Destroying Slam3 object");
		}

		void Shutdown(){
			if(mpORBSlam3){
				mpORBSlam3->Shutdown();
			}
		}
		cv::Mat GetCurrentFrame(){
			if(mpORBSlam3){
				return mpORBSlam3->GetCurrentFrame();
			}
			else{
				return cv::Mat();
			}
		};
		void TrackMonocular(Frame &frame, Sophus::SE3f &tcw);
		void InitialiseSlam(std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request, std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		void SetFrameMapPointUpdateCallback(std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> callback);

	private:
		// ORBSLAM3 Related pointers
		std::string mpVocabFilePath = "";
		std::string mpSettingsFilePath = "";
		ORB_SLAM3::System::eSensor mpCameraType;
		std::unique_ptr<ORB_SLAM3::System> mpORBSlam3 = nullptr;
		
		int GetTrackingState(){
			if(mpORBSlam3){
				return mpORBSlam3->GetTrackingState();
			}
			else{
				return -1;
			}
		};
};

// typedef MonoORBSLAM3 MonoORBSLAM3;
#endif
