#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;
using StartupSlam = slam::srv::StartupSlam;
using ShutdownSlam = std_srvs::srv::Trigger;

using namespace std;

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode()
:   Node("orbslam3_mono_node"),
{
	
	mpSlamStartupService = this->create_service<StartupSlam>("~/start_slam", &MonocularSlamNode::InitialiseSlamNode);
}

void MonocularSlamNode::InitialiseSlamNode(std::shared_ptr<slam::srv::start_slam::Request> request,
		std::shared_ptr<slam::srv::start_slam::Response> response){
	mpCameraTopicName = request->camera_topic;
	mpSlamSettingsFilePath = request->config_file_path;
	mpVocabFilePath = request->vocab_file_path;
	if(!mpSlam){
		mpSlam->Shutdown();
	}
	mpSlam = std::make_unique<ORB_SLAM3::System>(mpVocabFilePath,
			mpSlamSettingsFilePath,
			ORB_SLAM3::System::Monocular);
	mpFrameSubscriber = this->create_subscription<ImageMsg>(
			camera_topic,
			std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
	// mpAnnotatedFramePublisher = this->create_publisher<ImageMsg>("~/annotated_frame");
	// mpMapPublisher = this->create_publisher<MarkerMsg>("~/slam_map");

	mpState = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;

	// InitializeMarkersPublisher(mpSlamSettingsFilePath);
}


MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    mSLAM->Shutdown();
}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{

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
    Tcw = mSLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);

    // UpdateSLAMState();
    // UpdateMapState();
    //
    // PublishFrame();
    // PublishCurrentCamera();
    // PublishMapPoints();
    // PublishKeyFrames();
}


// void MonocularSlamNode::InitializeMarkersPublisher( const string &strSettingPath)
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
// void MonocularSlamNode::UpdateSLAMState()
// {
//  
//     unique_lock<mutex> lock(mMutex);
//     
//     ORB_SLAM3::Frame currentFrame = mSLAM->GetCurrentFrame();
//     mState = mSLAM->GetTrackingState();
//   
//     mvCurrentKeys = currentFrame.mvKeys;
//     N = mvCurrentKeys.size();
//     mvbVO = vector<bool>(N,false);
//     mvbMap = vector<bool>(N,false);
//     //mbOnlyTracking = pTracker->mbOnlyTracking;
//
//     if (mState == ORB_SLAM3::Tracking::NOT_INITIALIZED){
//         
//         mvIniKeys = mSLAM->GetInitialKeys();
//         mvIniMatches = mSLAM->GetInitialMatches();
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
// void MonocularSlamNode::UpdateMapState()
// {
//
//     unique_lock<mutex> lock(mMutex);
//
//     if (!Tcw.empty()){
//         mbCameraUpdated = true;
//
//     }
//
//     if (mSLAM->IsMapOptimized())
//     {
//         mbMapUpdated = true;
//      
//         mvKeyFrames = mSLAM->GetAllKeyFrames();
//         mvMapPoints = mSLAM->GetAllMapPoints();
//         mvRefMapPoints = mSLAM->GetReferenceMapPoints();
//
//     }
//
// }
//
//
//
// void MonocularSlamNode::PublishFrame()
// {
//     
//     cv::Mat im = DrawFrame();
//
//     cv_bridge::CvImage rosImage;
//     rosImage.image = im.clone();
//
//     rosImage.header.stamp = this->now();
//     rosImage.encoding = "bgr8";
//
//     mpAnnotatedFramePublisher->publish(rosImage.toImageMsg());
//     
// }
//
//
//
// cv::Mat MonocularSlamNode::DrawFrame()
// {
//     cv::Mat im;
//     vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
//     vector<int> vMatches; // Initialization: correspondeces with reference keypoints
//     vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
//     vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
//     int state; // Tracking state
//
//     //Copy variables within scoped mutex
//     {
//         unique_lock<mutex> lock(mMutex);
//         state=mState;
//         if(mState==ORB_SLAM3::Tracking::SYSTEM_NOT_READY)
//             mState=ORB_SLAM3::Tracking::NO_IMAGES_YET;
//
//         m_cvImPtr->image.copyTo(im);
//
//         if(mState==ORB_SLAM3::Tracking::NOT_INITIALIZED)
//         {
//             vCurrentKeys = mvCurrentKeys;
//             vIniKeys = mvIniKeys;
//             vMatches = mvIniMatches;
//         }
//         else if(mState==ORB_SLAM3::Tracking::OK)
//         {
//             vCurrentKeys = mvCurrentKeys;
//             vbVO = mvbVO;
//             vbMap = mvbMap;
//         }
//         else if(mState==ORB_SLAM3::Tracking::LOST)
//         {
//             vCurrentKeys = mvCurrentKeys;
//         }
//     } // destroy scoped mutex -> release mutex
//
//     if(im.channels()<3) //this should be always true
//         cvtColor(im,im,CV_GRAY2BGR);
//
//     //Draw
//     if(state==ORB_SLAM3::Tracking::NOT_INITIALIZED) //INITIALIZING
//     {
//         for(unsigned int i=0; i<vMatches.size(); i++)
//         {
//             if(vMatches[i]>=0)
//             {
//                 cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
//                         cv::Scalar(0,255,0));
//             }
//         }        
//     }
//     else if(state==ORB_SLAM3::Tracking::OK) //TRACKING
//     {
//         mnTracked=0;
//         mnTrackedVO=0;
//         const float r = 5;
//         const int n = vCurrentKeys.size();
//         for(int i=0;i<n;i++)
//         {
//             if(vbVO[i] || vbMap[i])
//             {
//                 cv::Point2f pt1,pt2;
//                 pt1.x=vCurrentKeys[i].pt.x-r;
//                 pt1.y=vCurrentKeys[i].pt.y-r;
//                 pt2.x=vCurrentKeys[i].pt.x+r;
//                 pt2.y=vCurrentKeys[i].pt.y+r;
//
//                 // This is a match to a MapPoint in the map
//                 if(vbMap[i])
//                 {
//                     cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
//                     cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
//                     mnTracked++;
//                 }
//                 else // This is match to a "visual odometry" MapPoint created in the last frame
//                 {
//                     cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
//                     cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
//                     mnTrackedVO++;
//                 }
//             }
//         }
//     }
//
//     cv::Mat imWithInfo;
//     DrawTextInfo(im,state, imWithInfo);
//
//     return imWithInfo;
//
// }
//
//
// void MonocularSlamNode::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
// {
//     stringstream s;
//     if(nState==ORB_SLAM3::Tracking::NO_IMAGES_YET)
//         s << " WAITING FOR IMAGES";
//     else if(nState==ORB_SLAM3::Tracking::NOT_INITIALIZED)
//         s << " TRYING TO INITIALIZE ";
//     else if(nState==ORB_SLAM3::Tracking::OK)
//     {
//         if(!mbOnlyTracking)
//             s << "SLAM MODE |  ";
//         else
//             s << "LOCALIZATION | ";
//         //int nKFs = mpMap->KeyFramesInMap();
//         //int nMPs = mpMap->MapPointsInMap();
//         int nKFs = 0;
//         int nMPs = 0;
//         s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
//         if(mnTrackedVO>0)
//             s << ", + VO matches: " << mnTrackedVO;
//     }
//     else if(nState==ORB_SLAM3::Tracking::LOST)
//     {
//         s << " TRACK LOST. TRYING TO RELOCALIZE ";
//     }
//     else if(nState==ORB_SLAM3::Tracking::SYSTEM_NOT_READY)
//     {
//         s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
//     }
//
//     int baseline=0;
//     cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
//
//     imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
//     im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
//     imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
//     cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
//
// }
//
//
//
// void MonocularSlamNode::PublishCurrentCamera()
// {
//
//
//
//
//
//
// }
//
//
//
// void MonocularSlamNode::PublishMapPoints()
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
//
//
//     mPoints.header.stamp = this->now();
//     mReferencePoints.header.stamp = this->now();
//     m_map_publisher->publish(mPoints);
//     m_map_publisher->publish(mReferencePoints);
//
// }
//
//
// void MonocularSlamNode::PublishKeyFrames()
// {
//
//
//
//
//
//
// }
