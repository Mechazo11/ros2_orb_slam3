#include "slam/orbslam3/monocular.hpp"

#include <opencv2/core/core.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

using namespace std;

using std::placeholders::_1;

ORBSLAM3SlamNode::ORBSLAM3SlamNode()
:   SlamNode("orbslam3_mono_node")
{	
	mpSlamStartupService = this->create_service<StartupSlam>("~/start_slam", std::bind(&ORBSLAM3SlamNode::InitialiseSlamNode, this, std::placeholders::_1, std::placeholders::_2));
	mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(tf2_ros::TransformBroadcaster(this)); 
}

void ORBSLAM3SlamNode::InitialiseSlamNode(std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){
	SlamNode::InitialiseSlamNode(request, response);
	// TODO call parent class method of same name here
	RCLCPP_INFO(this->get_logger(), "Creating Frame Subscription for Tracking");
	mpFrameSubscriber = this->create_subscription<ImageMsg>(
			mpCameraTopicName,
			10,
			std::bind(&ORBSLAM3SlamNode::GrabImage, this, std::placeholders::_1));
	mpAnnotatedFramePublisher = this->create_publisher<ImageMsg>("~/annotated_frame", 10);
	// mpMapPublisher = this->create_publisher<MarkerMsg>("~/slam_map");

	mpState = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;
	response->success = true;
	response->message = "Successfully created SLAM object and required subscribers";
	RCLCPP_INFO(this->get_logger(), "ORBSLAM3 initialisation complete");

	// InitializeMarkersPublisher(mpSlamSettingsFilePath);
}


ORBSLAM3SlamNode::~ORBSLAM3SlamNode()
{
    // Stop all threads
    mpSlam->Shutdown();
}


void ORBSLAM3SlamNode::GrabImage(const ImageMsg::SharedPtr msg)
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
	mpSlam->TrackMonocular(frame, mpTcw);
	PublishFrame();
	// Publishes all data common to all SLAM algorithms
	Update();
    
    // UpdateSLAMState();
    // UpdateMapState();
    //
    // PublishCurrentCamera();
    // PublishMapPoints();
    // PublishKeyFrames();
}

void ORBSLAM3SlamNode::PublishFrame()
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



// void ORBSLAM3SlamNode::InitializeMarkersPublisher( const string &strSettingPath)
// {
//
//     cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//
//     mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
//     mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
//     mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
//     mPointSize = fSettings["Viewer.PointSize"];
//     mCameraSize = fSettings["Viewer.CameraSize"];
//     mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
//
//
//     const char* MAP_FRAME_ID = "map";
//     const char* POINTS_NAMESPACE = "MapPoints";
//     const char* KEYFRAMES_NAMESPACE = "KeyFrames";
//     const char* GRAPH_NAMESPACE = "Graph";
//     const char* CAMERA_NAMESPACE = "Camera";
//     
//     //Configure MapPoints
//     mPoints.header.frame_id = MAP_FRAME_ID;
//     mPoints.ns = POINTS_NAMESPACE;
//     mPoints.id=0;
//     mPoints.type = MarkerMsg::POINTS;
//     mPoints.scale.x=mPointSize;
//     mPoints.scale.y=mPointSize;
//     mPoints.pose.orientation.w=1.0;
//     mPoints.action=MarkerMsg::ADD;
//     mPoints.color.a = 1.0;
//
//     //Configure KeyFrames
//     mCameraSize=0.04;
//     mKeyFrames.header.frame_id = MAP_FRAME_ID;
//     mKeyFrames.ns = KEYFRAMES_NAMESPACE;
//     mKeyFrames.id=1;
//     mKeyFrames.type = MarkerMsg::LINE_LIST;
//     mKeyFrames.scale.x=0.005;
//     mKeyFrames.pose.orientation.w=1.0;
//     mKeyFrames.action=MarkerMsg::ADD;
//
//     mKeyFrames.color.b=1.0f;
//     mKeyFrames.color.a = 1.0;
//
//     //Configure Covisibility Graph
//     mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
//     mCovisibilityGraph.ns = GRAPH_NAMESPACE;
//     mCovisibilityGraph.id=2;
//     mCovisibilityGraph.type = MarkerMsg::LINE_LIST;
//     mCovisibilityGraph.scale.x=0.002;
//     mCovisibilityGraph.pose.orientation.w=1.0;
//     mCovisibilityGraph.action=MarkerMsg::ADD;
//     mCovisibilityGraph.color.b=0.7f;
//     mCovisibilityGraph.color.g=0.7f;
//     mCovisibilityGraph.color.a = 0.3;
//
//     //Configure KeyFrames Spanning Tree
//     mMST.header.frame_id = MAP_FRAME_ID;
//     mMST.ns = GRAPH_NAMESPACE;
//     mMST.id=3;
//     mMST.type = MarkerMsg::LINE_LIST;
//     mMST.scale.x=0.005;
//     mMST.pose.orientation.w=1.0;
//     mMST.action=MarkerMsg::ADD;
//     mMST.color.b=0.0f;
//     mMST.color.g=1.0f;
//     mMST.color.a = 1.0;
//
//     //Configure Current Camera
//     mCurrentCamera.header.frame_id = MAP_FRAME_ID;
//     mCurrentCamera.ns = CAMERA_NAMESPACE;
//     mCurrentCamera.id=4;
//     mCurrentCamera.type = MarkerMsg::LINE_LIST;
//     mCurrentCamera.scale.x=0.01;//0.2; 0.03
//     mCurrentCamera.pose.orientation.w=1.0;
//     mCurrentCamera.action=MarkerMsg::ADD;
//     mCurrentCamera.color.g=1.0f;
//     mCurrentCamera.color.a = 1.0;
//     
//     //Configure Reference MapPoints
//     mReferencePoints.header.frame_id = MAP_FRAME_ID;
//     mReferencePoints.ns = POINTS_NAMESPACE;
//     mReferencePoints.id=6;
//     mReferencePoints.type = MarkerMsg::POINTS;
//     mReferencePoints.scale.x=mPointSize;
//     mReferencePoints.scale.y=mPointSize;
//     mReferencePoints.pose.orientation.w=1.0;
//     mReferencePoints.action=MarkerMsg::ADD;
//     mReferencePoints.color.r =1.0f;
//     mReferencePoints.color.a = 1.0;
//
//
//     mpMapPublisher->publish(mPoints);
//     mpMapPublisher->publish(mReferencePoints);
//     //m_map_publisher->publish(mCovisibilityGraph);
//     mpMapPublisher->publish(mKeyFrames);
//     //m_map_publisher->publish(mCurrentCamera);
//
//
// }
//
//
// void ORBSLAM3SlamNode::UpdateSLAMState()
// {
//  
//     unique_lock<mutex> lock(mMutex);
//     
//     ORB_SLAM3::Frame currentFrame = mpSlam->GetCurrentFrame();
//     mState = mpSlam->GetTrackingState();
//   
//     mvCurrentKeys = currentFrame.mvKeys;
//     N = mvCurrentKeys.size();
//     mvbVO = vector<bool>(N,false);
//     mvbMap = vector<bool>(N,false);
//     //mbOnlyTracking = pTracker->mbOnlyTracking;
//
//     if (mState == ORB_SLAM3::Tracking::NOT_INITIALIZED){
//         
//         mvIniKeys = mpSlam->GetInitialKeys();
//         mvIniMatches = mpSlam->GetInitialMatches();
//     }
//     else if(mState == ORB_SLAM3::Tracking::OK)
//     {
//         for(int i=0;i<N;i++)
//         {
//             ORB_SLAM3::MapPoint* pMP = currentFrame.mvpMapPoints[i];
//             if(pMP)
//             {
//                 if(!currentFrame.mvbOutlier[i])
//                 {
//                     if(pMP->Observations()>0)
//                         mvbMap[i]=true;
//                     else
//                         mvbVO[i]=true;
//                 }
//             }
//         }
//     }
//
//     mbUpdated = true;
//     
// }
//
//
// void ORBSLAM3SlamNode::UpdateMapState()
// {
//
//     unique_lock<mutex> lock(mMutex);
//
//     if (!Tcw.empty()){
//         mbCameraUpdated = true;
//
//     }
//
//     if (mpSlam->IsMapOptimized())
//     {
//         mbMapUpdated = true;
//      
//         mvKeyFrames = mpSlam->GetAllKeyFrames();
//         mvMapPoints = mpSlam->GetAllMapPoints();
//         mvRefMapPoints = mpSlam->GetReferenceMapPoints();
//
//     }
//
// }
//
//
//
//
//
//
//
//
//
//
//
//
// void ORBSLAM3SlamNode::PublishMapPoints()
// {
//
//     mPoints.points.clear();
//     mReferencePoints.points.clear();
//
//     set<ORB_SLAM3::MapPoint*> spRefMPs(mvRefMapPoints.begin(), mvRefMapPoints.end());
//
//     if(mvMapPoints.empty())
//         return;
//
//     for(size_t i=0, iend=mvMapPoints.size(); i<iend;i++)
//     {
//         if(mvMapPoints[i]->isBad() || spRefMPs.count(mvMapPoints[i]))
//             continue;
//         PointMsg p;
//         cv::Mat pos = mvMapPoints[i]->GetWorldPos();
//         p.x=pos.at<float>(0);
//         p.y=pos.at<float>(1);
//         p.z=pos.at<float>(2);
//
//         mPoints.points.push_back(p);
//     }
//
//     for(set<ORB_SLAM3::MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
//     {
//         if((*sit)->isBad())
//             continue;
//         PointMsg p;
//         cv::Mat pos = (*sit)->GetWorldPos();
//         p.x=pos.at<float>(0);
//         p.y=pos.at<float>(1);
//         p.z=pos.at<float>(2);
//
//         mReferencePoints.points.push_back(p);
//
//     }


    // mPoints.header.stamp = this->now();
    // mReferencePoints.header.stamp = this->now();
    // m_map_publisher->publish(mPoints);
    // m_map_publisher->publish(mReferencePoints);

// }
//
//
// void ORBSLAM3SlamNode::PublishKeyFrames()
// {
//
//
//
//
//
//
// }
