#ifndef ORB_SLAM3_NODE_HPP
#define ORB_SLAM3_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "slam/slam.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include "tf2/utils.h"
// #include "geometry_msgs/msgs/transform_stamped.hpp"

template<eSlamType slamType>
class SlamNode : public rclcpp::Node{
	public:
		SlamNode(std::string nodeName);
		Sophus::SE3f mpTcw;
	protected:
		void Update();
		std::unique_ptr<tf2_ros::TransformBroadcaster> mpTfBroadcaster;
		void InitialiseSlamNode(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		std::unique_ptr<Slam<slamType>> mpSlam;
	private:
		std::string mpSlamType = "";
		std::string mpCameraTopicName = "";
		std::string mpSlamSettingsFilePath = "";
		std::string mpVocabFilePath = "";

		// Publication Callbacks
		void PublishPositionAsTransform(Sophus::SE3f &Tcw);
		tf2::Transform TransformFromMat(cv::Mat position_mat);
		tf2::Transform TransformToTarget(tf2::Transform tf_in, std::string frame_in, std::string frame_target);
		// void PublishPositionAsPoseStamped(Sophus::SE3f Tcw);
		void PublishState(int trackingState);
		// TODO create an abstraction for map points
		// void PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points);
		// void PublishGBAStatus (bool gba_status);
		// void PublishRenderedImage (Frame frame);
		// void publishVertices(std::list<float>& l);
		// void publishEdges(std::list<float>& l);
		// void publishPoints(std::list<float>& l);

		// Startup/Shutdown Services
		rclcpp::Service<custom_interfaces::srv::StartupSlam>::SharedPtr mpSlamStartupService;

		// Default Publishers and Subscribers

};

template class SlamNode<eSlamType::ORBSLAM3>;

#endif
