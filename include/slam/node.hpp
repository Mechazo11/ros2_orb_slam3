#ifndef SLAM_NODE_HPP
#define SLAM_NODE_HPP

#include "slam/slam.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

bool readYAMLFile(std::string &yamlPath, YAML::Node &output);

class SlamNode : public rclcpp::Node{
	public:
		SlamNode(std::string nodeName);
		Sophus::SE3f mpTwc;
	protected:
		void Update();
		std::unique_ptr<tf2_ros::TransformBroadcaster> mpTfBroadcaster;
		void InitialiseSlamNode(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		std::string mpCameraTopicName = "";
		std::string mpSlamConfigFilePath = "";
	private:

		// Publication Callbacks
		void PublishPositionAsTransform(Sophus::SE3f &Tcw);
		void PublishState(int trackingState);

		// Startup/Shutdown Services
		rclcpp::Service<custom_interfaces::srv::StartupSlam>::SharedPtr mpSlamStartupService;
};

#endif
