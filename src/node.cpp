#include "slam/node.hpp"

geometry_msgs::msg::Transform sophusToTransformMsg(Sophus::SE3f& se3) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = se3.translation().x();
  msg.translation.y = se3.translation().y();
  msg.translation.z = se3.translation().z();
  msg.rotation.x = se3.unit_quaternion().x();
  msg.rotation.y = se3.unit_quaternion().y();
  msg.rotation.z = se3.unit_quaternion().z();
  msg.rotation.w = se3.unit_quaternion().w();
  return msg;
}

SlamNode::SlamNode(std::string nodeName)
:	rclcpp::Node(nodeName){
	RCLCPP_INFO(this->get_logger(), "Initialising Slam Node");
}

void SlamNode::InitialiseSlamNode(std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){
	RCLCPP_INFO(this->get_logger(), "Initialising Base Node");
}

void SlamNode::Update(){
	PublishPositionAsTransform(mpTwc);
}

void SlamNode::PublishPositionAsTransform(Sophus::SE3f &tcw){
	// Get transform from slam to camera frame as a message
	geometry_msgs::msg::TransformStamped msg;
	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "map";
	msg.child_frame_id = "camera";
	msg.transform  = sophusToTransformMsg(tcw); 
	// Broadcast tf                                                       
	mpTfBroadcaster->sendTransform(msg);
}
