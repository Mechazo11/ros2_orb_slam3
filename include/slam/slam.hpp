#ifndef SLAM_OBJECT_HPP
#define SLAM_OBJECT_HPP

// ORBSLAM3 Dependencies
#include"System.h"
#include "Map.h"
#include <opencv2/core/core.hpp>
#include <memory>
#include <thread>
#include "custom_interfaces/srv/startup_slam.hpp"
#include "std_srvs/srv/trigger.hpp"

using StartupSlam = custom_interfaces::srv::StartupSlam;
using ShutdownSlam = std_srvs::srv::Trigger;

class Frame{
	public:
		Frame(std::shared_ptr<cv::Mat> image, long &timestamp);
		cv::Mat getImage(){
			return *(mpImage.get());
		}
		long getTimestampMSec(){
			return mpTimestamp * 1e-6;
		}
		long getTimestampSec(){
			return mpTimestamp * 1e-9;
		}
		Sophus::SE3f getTransform(){
			return Tcw;
		}
	private:
		// TODO not a good idea to have nested threadsafe. this must be used for a collection of data always to be used in one thread at a time.
		std::shared_ptr<cv::Mat> mpImage;
		
		long mpTimestamp;

		Sophus::SE3f Tcw;
};
	
enum eSlamType{
	NOT_SET=-1,
	ORBSLAM3=0
};

template<eSlamType slamType>
class Slam{
	public:
		Slam(){
			mpSlam = nullptr;
		};
		void InitialiseSlam(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		void Shutdown(){
			if(slamType == eSlamType::ORBSLAM3){
				mpSlam->Shutdown();
			}
		}

		cv::Mat GetCurrentFrame(){
			return mpSlam->GetCurrentFrame();
		}

		void TrackMonocular(Frame &frame, Sophus::SE3f &tcw);
	private:
		eSlamType mpSlamType = slamType;
		// ORBSLAM3 Related pointers
		std::unique_ptr<ORB_SLAM3::System> mpSlam = nullptr;
		int GetTrackingState();
		Sophus::SE3f GetCurrentPosition();
		
};

template class Slam<eSlamType::ORBSLAM3>;

#endif
