#ifndef MONOCULAR_SLAM_NODE_HPP
#define MONOCULAR_SLAM_NODE_HPP


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/Trigger"
#include "slam/srv/StartSlam.hpp"

#include <cv_bridge/cv_bridge.h>
// ORB_SLAM3 related includes
#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "MapPoint.h"

class MonocularSlamNode : public rclcpp::Node
{

public:

    MonocularSlamNode(std::string node_name);


    ~MonocularSlamNode();


private: 
	// Node Configurations
	std::string mpCameraTopicName;
	std::string mpSlamSettingsFilePath;
	std::string mpVocabFilePath;

	// ORB_SLAM3 related attributes
	std::unique_ptr<ORB_SLAM3::System> mpSlam = std::make_unique(nullptr);
	ORB_SLAM3::eTrackingState mpState;
	// Image pointer for receiving and passing images to SLAM
    cv_bridge::CvImagePtr m_cvImPtr;
	// Output of Tracking method
    cv::Mat Tcw;
	
	// Publishers, Subscribers, Services and Actions
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpAnnotatedFramePublisher;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpMapPublisher;
	rclcpp::Service<slam::srv::StartupSlam>::SharedPtr mpSlamStartupService
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mpSlamShutdownService

	// Callbacks and Methods
	void InitialiseSlamNode(std::shared_ptr<slam::srv::start_slam::Request> request,
		std::shared_ptr<slam::srv::start_slam::Response> response);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    // void UpdateSLAMState();
    //
    // void UpdateMapState();
    //
    //
    // void PublishFrame();
    //
    // void PublishCurrentCamera();
    //
    // void InitializeMarkersPublisher( const string &strSettingPath);
    // void PublishMapPoints();
    // void PublishKeyFrames();
    //
    //
    // cv::Mat DrawFrame();
    //
    // void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    //
    //
    //
    //
    // std::mutex mMutex;

    
    
    // int N;
    // vector<cv::KeyPoint> mvCurrentKeys;
    // vector<bool> mvbMap, mvbVO;
    // int mnTracked, mnTrackedVO;
    // vector<cv::KeyPoint> mvIniKeys;
    // vector<int> mvIniMatches;
    //
    //
    // vector<ORB_SLAM3::KeyFrame*> mvKeyFrames;
    // vector<ORB_SLAM3::MapPoint*> mvMapPoints;
    // vector<ORB_SLAM3::MapPoint*> mvRefMapPoints;
    //
    // visualization_msgs::msg::Marker mPoints;
    // visualization_msgs::msg::Marker mReferencePoints;
    // visualization_msgs::msg::Marker mKeyFrames;
    // visualization_msgs::msg::Marker mReferenceKeyFrames;
    // visualization_msgs::msg::Marker mCovisibilityGraph;
    // visualization_msgs::msg::Marker mMST;
    // visualization_msgs::msg::Marker mCurrentCamera;
    //
    // float mKeyFrameSize;
    // float mKeyFrameLineWidth;
    // float mGraphLineWidth;
    // float mPointSize;
    // float mCameraSize;
    // float mCameraLineWidth;
    //
    // bool mbOnlyTracking;
    // bool mbUpdated;
    // bool mbCameraUpdated;
    // bool mbMapUpdated;


};


#endif
