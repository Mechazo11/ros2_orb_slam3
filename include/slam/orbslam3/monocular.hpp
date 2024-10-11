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

class MonocularSlamNode : public SlamNode
{
	public:
		MonocularSlamNode();
		~MonocularSlamNode();
	private:
		// Node Configurations
		std::string mpCameraTopicName;
		std::string mpSlamSettingsFilePath;
		std::string mpVocabFilePath;
		std::unique_ptr<Slam> mpSlam = nullptr;
		// ORB_SLAM3 related attributes
		ORB_SLAM3::Tracking::eTrackingState mpState;
		// Image pointer for receiving and passing images to SLAM
		cv_bridge::CvImagePtr m_cvImPtr;
		// Publishers, Subscribers, Services and Actions
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpAnnotatedFramePublisher;
		rclcpp::Service<custom_interfaces::srv::StartupSlam>::SharedPtr mpSlamStartupService;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mpSlamShutdownService;
		// Callbacks and Methods
		void Update();
		void InitialiseSlamNode(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
		void PublishFrame();
};
#endif
