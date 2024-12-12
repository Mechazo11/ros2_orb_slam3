#include "slam/monocular.hpp"

MonocularSlamNode::MonocularSlamNode() : SlamNode("orbslam3_mono_node"){
	RCLCPP_INFO(this->get_logger(), "Creating SLAM Object in Mono Slam Node");
#ifdef USE_ORBSLAM3
	mpSlam = std::unique_ptr<Slam>(new MonoORBSLAM3(this->get_logger()));
	RCLCPP_INFO(this->get_logger(), "Created SLAM Object for ORBSLAM3 in Mono Slam Node");
#endif
#ifdef USE_MORBSLAM
	mpSlam = std::unique_ptr<Slam>(new MonoMORBSLAM(this->get_logger()));
	RCLCPP_INFO(this->get_logger(), "Created SLAM Object for MORBSLAM in Mono Slam Node");
#endif
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
	mpMapPointPublisher = this->create_publisher<MapMsg>("~/map_points", 10);
	// mpMapPublisher = this->create_publisher<MarkerMsg>("~/slam_map");
	mpSlam->InitialiseSlam(request, response);

#ifdef USE_ORBSLAM3
	std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> cb = [this](std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
		PublishMapPointsCallback(mapPoints, tcw);
	};
	mpSlam->SetFrameMapPointUpdateCallback(cb);
	mpState = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;
#endif
#ifdef USE_MORBSLAM
	mpState = MORB_SLAM::TrackingState::SYSTEM_NOT_READY;
#endif
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
   	mpCurrentFrame = Frame(std::make_shared<cv::Mat>(m_cvImPtr->image), timestamp);
	Sophus::SE3f tcw;
	mpSlam->TrackMonocular(mpCurrentFrame, tcw);
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

#ifdef USE_ORBSLAM3
void MonocularSlamNode::PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
	if(mapPoints.size() == 0){
		RCLCPP_ERROR(this->get_logger(), "No Map Points to add to octomap");
		return;
	}
	MapMsg msg = MapPointsToPointCloud(mapPoints, tcw);
	mpMapPointPublisher->publish(msg);
}

MapMsg MonocularSlamNode::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
    MapMsg cloud;

    const int num_channels = 3; // x y z
	long current_frame_time_ = mpCurrentFrame.getTimestampNSec();
	Eigen::Matrix3d cam_base_R_ = tcw.so3().matrix().cast<double>();
	Eigen::Vector3d cam_base_T_ = tcw.translation().cast<double>();

    cloud.header.stamp.sec = static_cast<int32_t>(current_frame_time_ / 1e9);
	cloud.header.stamp.nanosec = static_cast<int32_t>(current_frame_time_ % static_cast<long long>(1e9));
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = mapPoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i=0; i<cloud.width; i++) {
        if(!mapPoints[i])
            continue;

        if (mapPoints.at(i)->nObs >= mpNumMinObsPerPoint) {
            Eigen::Vector3f map_pt_f = mapPoints.at(i)->GetWorldPos();

            Eigen::Vector3d map_pt = map_pt_f.cast<double>();

            map_pt = cam_base_R_ * map_pt;
            map_pt += cam_base_T_;

            data_array[0] = map_pt[0];
            data_array[1] = map_pt[1];
            data_array[2] = map_pt[2];

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return cloud;
}

#endif
