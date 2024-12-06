#ifndef SLAM_OBJECT_HPP
#define SLAM_OBJECT_HPP

// ORBSLAM3 Dependencies
#include <opencv2/core/core.hpp>
#include <memory>
#include <thread>
#include <exception>
#include "custom_interfaces/srv/startup_slam.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sophus/se3.hpp>

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

class Slam{
	public:
		Slam(rclcpp::Logger logger);
		virtual void InitialiseSlam(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response) = 0;
		virtual void Shutdown() = 0;
		virtual cv::Mat GetCurrentFrame() = 0;
		virtual void TrackMonocular(Frame &frame, Sophus::SE3f &tcw) = 0;
		rclcpp::Logger mpLogger;
	private:	
		// Getter Methods
		virtual int GetTrackingState() = 0;
};

#endif
