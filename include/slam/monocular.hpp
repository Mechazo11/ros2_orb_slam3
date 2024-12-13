#include "slam/node.hpp"
#ifdef USE_ORBSLAM3
#include "slam/orbslam3/monocular.hpp"
#endif

#ifdef  USE_MORBSLAM
#include "slam/morbslam/monocular.hpp"
#endif

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
		Frame mpCurrentFrame;
		std::unique_ptr<Slam> mpSlam = nullptr;
		// ORB_SLAM3 related attributes
#ifdef USE_ORBSLAM3
		ORB_SLAM3::Tracking::eTrackingState mpState;
#endif
#ifdef USE_MORBSLAM
		MORB_SLAM::TrackingState mpState = MORB_SLAM::TrackingState::SYSTEM_NOT_READY;
#endif
		// Image pointer for receiving and passing images to SLAM
		cv_bridge::CvImagePtr m_cvImPtr;
		// Publishers, Subscribers, Services and Actions
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpAnnotatedFramePublisher;
		rclcpp::Publisher<MapMsg>::SharedPtr mpMapPointPublisher;
		rclcpp::Service<custom_interfaces::srv::StartupSlam>::SharedPtr mpSlamStartupService;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mpSlamShutdownService;
		// Callbacks and Methods
		void Update();
		void InitialiseSlamNode(
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
				std::shared_ptr<custom_interfaces::srv::StartupSlam::Response> response);
		void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
		void PublishFrame();
#ifdef USE_ORBSLAM3
		void PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		MapMsg MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		// TODO need to make this a parameter
		int mpNumMinObsPerPoint = 2;
#endif
};
