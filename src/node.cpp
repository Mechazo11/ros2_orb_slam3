#include "slam/node.hpp"

template<eSlamType slamType>
SlamNode<slamType>::SlamNode(std::string nodeName)
:	rclcpp::Node(nodeName){
	RCLCPP_INFO(this->get_logger(), "Initialising Slam Node");
}

template<eSlamType slamType>
void SlamNode<slamType>::InitialiseSlamNode(std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){

	RCLCPP_INFO(this->get_logger(), "Setting ORBSLAM3 and initiasing node");
	mpCameraTopicName = request->camera_topic;
	mpSlamSettingsFilePath = request->config_file_path;

	std::string camera_topic_message = "Got Camera topic name: " + mpCameraTopicName;
	RCLCPP_INFO(this->get_logger(), camera_topic_message.c_str());

	std::string settings_file_path_message = "Got Settings file path: " + mpSlamSettingsFilePath;
	RCLCPP_INFO(this->get_logger(), settings_file_path_message.c_str());
	
	std::string vocab_file_path_message = "Got Vocab file path: " + mpVocabFilePath;
	RCLCPP_INFO(this->get_logger(), vocab_file_path_message.c_str());

	if(mpSlam){
		mpSlam->Shutdown();
	}
	RCLCPP_INFO(this->get_logger(), "Creating ORBSLAM3 object");
	if(mpSlamType == "orbslam3"){
		mpSlam->InitialiseSlam(request, response);
	}
	// TODO need to check if we can directly start the subscriber just after creating the slam object
	response->success = true;
	response->message = "Successfully created SLAM object and required subscribers";
	RCLCPP_INFO(this->get_logger(), "ORBSLAM3 initialisation complete");

	// InitializeMarkersPublisher(mpSlamSettingsFilePath);
}

template<eSlamType slamType>
void SlamNode<slamType>::Update(){
	PublishPositionAsTransform(mpTcw);
}

template<eSlamType slamType>
void SlamNode<slamType>::PublishPositionAsTransform(Sophus::SE3f &tcw) {
	// Get transform from map to camera frame                             
	cv::Mat Tcw = cv::Mat(4, 4, CV_32F, tcw.matrix().data());
	tf2::Transform tf_transform = this->TransformFromMat(Tcw);             

	// Make transform from camera frame to target frame
	// TODO enable the aformentioned code as and when needed, currently broadcasting transform from slam as is
	// tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id_param_, target_frame_id_param_);
	tf2::Transform tf_map2target = tf_transform;

	// Make message                                                       
	// tf2::Stamped<tf2::Transform> tf_map2target_stamped;                   
	// std::string map_frame_id_param_ = "map";
	// tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, this->now(), map_frame_id_param_);
	geometry_msgs::msg::TransformStamped msg;
	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "map";
	msg.child_frame_id = "slam";
	msg.transform = tf2::toMsg(tf_map2target);
	// msg.child_frame_id = target_frame_id_param_;                          
	// Broadcast tf                                                       
	mpTfBroadcaster->sendTransform(msg);
}

template<eSlamType slamType>
tf2::Transform SlamNode<slamType>::TransformFromMat (cv::Mat position_mat) {
	cv::Mat rotation(3,3,CV_32F);
	cv::Mat translation(3,1,CV_32F);

	rotation = position_mat.rowRange(0,3).colRange(0,3);
	translation = position_mat.rowRange(0,3).col(3);


	tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
			rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
			rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
			);

	tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

	//Coordinate transformation matrix from orb coordinate system to ros coordinate system
	const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
			-1, 0, 0,
			0,-1, 0);

	//Transform from orb coordinate system to ros coordinate system on camera coordinates
	tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
	tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

	//Inverse matrix
	tf_camera_rotation = tf_camera_rotation.transpose();
	tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

	//Transform from orb coordinate system to ros coordinate system on map coordinates
	tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
	tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

	return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

// template<eSlamType slamType>
// tf2::Transform SlamNode<slamType>::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
// 	// Transform tf_in from frame_in to frame_target
// 	tf2::Transform tf_map2orig = tf_in;
// 	tf2::Transform tf_orig2target;
// 	tf2::Transform tf_map2target;
//
// 	tf2::Stamped<tf2::Transform> transformStamped_temp;
// 	try {
// 		// Get the transform from camera to target
// 		geometry_msgs::msg::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
// 		// Convert to tf2
// 		tf2::fromMsg(tf_msg, transformStamped_temp);
// 		tf_orig2target.setBasis(transformStamped_temp.getBasis());
// 		tf_orig2target.setOrigin(transformStamped_temp.getOrigin());
//
// 	} catch (tf2::TransformException &ex) {
// 		RCLCPP_WARN(this->get_logger(), "%s",ex.what());
// 		//ros::Duration(1.0).sleep();
// 		tf_orig2target.setIdentity();
// 	}
//
// 	// Transform from map to target
// 	tf_map2target = tf_map2orig * tf_orig2target;
//
// 	#<{(|
// 	// Print debug info
// 	double roll, pitch, yaw;
// 	// Print debug map2orig
// 	tf2::Matrix3x3(tf_map2orig.getRotation()).getRPY(roll, pitch, yaw);
// 	ROS_INFO("Static transform Map to Orig [%s -> %s]",
// 	map_frame_id_param_.c_str(), frame_in.c_str());
// 	ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
// 	tf_map2orig.getOrigin().x(), tf_map2orig.getOrigin().y(), tf_map2orig.getOrigin().z());
// 	ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
// 	RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
// 	// Print debug tf_orig2target
// 	tf2::Matrix3x3(tf_orig2target.getRotation()).getRPY(roll, pitch, yaw);
// 	ROS_INFO("Static transform Orig to Target [%s -> %s]",
// 	frame_in.c_str(), frame_target.c_str());
// 	ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
// 	tf_orig2target.getOrigin().x(), tf_orig2target.getOrigin().y(), tf_orig2target.getOrigin().z());
// 	ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
// 	RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
// 	// Print debug map2target
// 	tf2::Matrix3x3(tf_map2target.getRotation()).getRPY(roll, pitch, yaw);
// 	ROS_INFO("Static transform Map to Target [%s -> %s]",
// 	map_frame_id_param_.c_str(), frame_target.c_str());
// 	ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
// 	tf_map2target.getOrigin().x(), tf_map2target.getOrigin().y(), tf_map2target.getOrigin().z());
// 	ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
// 	RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
//
// |)}>#
// 	return tf_map2target;
// }
