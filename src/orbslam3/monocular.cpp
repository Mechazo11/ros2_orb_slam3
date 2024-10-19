#include "slam/orbslam3/monocular.hpp"

#include <opencv2/core/core.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

using namespace std;

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode() : SlamNode("orbslam3_mono_node"){
	RCLCPP_INFO(this->get_logger(), "Creating SLAM Object in Mono Slam Node");
	mpSlam = std::unique_ptr<Slam>(new MonoORBSLAM3(this->get_logger()));
	RCLCPP_INFO(this->get_logger(), "Created SLAM Object in Mono Slam Node");
	mpSlamStartupService = this->create_service<StartupSlam>("~/start_slam", std::bind(&MonocularSlamNode::InitialiseSlamNode, this, std::placeholders::_1, std::placeholders::_2));
	mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(tf2_ros::TransformBroadcaster(this)); 
}

void MonocularSlamNode::InitialiseSlamNode(std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){
	SlamNode::InitialiseSlamNode(request, response);
	// TODO call parent class method of same name here
	mpCameraTopicName = request->camera_topic;
	mpSlamConfigFilePath = request->config_file_path;
	RCLCPP_INFO(this->get_logger(), "Got Camera topic name: %s", mpCameraTopicName.c_str());
	RCLCPP_INFO(this->get_logger(), "Got Config file path: %s", mpSlamConfigFilePath.c_str());	
	RCLCPP_INFO(this->get_logger(), "Creating Frame Subscription for topic: %s perform to visual odometry", mpCameraTopicName.c_str());
	mpFrameSubscriber = this->create_subscription<ImageMsg>(
			mpCameraTopicName,
			10,
			std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
	mpAnnotatedFramePublisher = this->create_publisher<ImageMsg>("~/annotated_frame", 10);
	// mpMapPublisher = this->create_publisher<MarkerMsg>("~/slam_map");
	mpSlam->InitialiseSlam(request, response);

	mpState = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;
	if(!response->success){
		response->success = true;
		response->message = "Successfully created SLAM object and required subscribers";
		RCLCPP_INFO(this->get_logger(), "ORBSLAM3 initialisation complete");
	}
	else{
		RCLCPP_ERROR(this->get_logger(), "ORBSLAM3 initialisation failed");
	}

	// InitializeMarkersPublisher(mpSlamSettingsFilePath);
}


void MonocularSlamNode::Update(){
	SlamNode::Update();
	PublishFrame();
}


MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    mpSlam->Shutdown();
}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
	RCLCPP_DEBUG(this->get_logger(), "Received Image Message for Tracking");

    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
	// TODO check what kind of buffering and mutex is needed for performing SLAM tracking
	int sec = msg->header.stamp.sec;
	int nsec = msg->header.stamp.nanosec;
	long timestamp = sec * 1e9 + nsec;
   	Frame frame = Frame(std::make_shared<cv::Mat>(m_cvImPtr->image), timestamp);
	Sophus::SE3f tcw;
	mpSlam->TrackMonocular(frame, tcw);
	// The output of ORBSLAM3 is the transformation that can convert points
	// from world frame to camera frame. Thus, Tcw * Xw would be equivalent to
	// Xw to Xc. To get the position of the camera with respect to slam origin,
	// the transformation needs to be inverted to obtain camera position in 
	// world reference
	// The following link documents an issue related to this in ORBSLAM2 repo
	// https://github.com/raulmur/ORB_SLAM2/issues/226
	// One may also refer to the following link to check the implementation of
	// of inverse for Sophus::SE3f 
	mpTwc = tcw.inverse();
	// Publishes all data common to all SLAM algorithms
	Update();
    
    // UpdateSLAMState();
    // UpdateMapState();
    //
    // PublishCurrentCamera();
    // PublishMapPoints();
    // PublishKeyFrames();
}

void MonocularSlamNode::PublishFrame()
{

	cv::Mat drawnFrame = mpSlam->GetCurrentFrame();
    // cv::Mat im = DrawFrame();

	sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
	cv_bridge::CvImage(
			std_msgs::msg::Header(),
			sensor_msgs::image_encodings::BGR8,
			drawnFrame.clone()
			).toImageMsg(*img_msg_ptr);
	img_msg_ptr->header.stamp = this->now();
	// img_msg_ptr->height = frame.height;
	// img_msg_ptr->width = frame.width;
	// img_msg_ptr->is_bigendian = false;
	// img_msg_ptr->step = frame.width * frame.frame.elemSize();

    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr->image = drawnFrame.clone();
    //
    // cv_ptr->header.stamp = this->now();
    // cv_ptr->encoding = "bgr8";

    mpAnnotatedFramePublisher->publish(std::move(img_msg_ptr));

}
