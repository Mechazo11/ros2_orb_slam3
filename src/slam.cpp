#include "slam/slam.hpp"

Frame::Frame(std::shared_ptr<cv::Mat> image, long &timestamp){
	mpImage = image;
	mpTimestamp = timestamp;
}

template<eSlamType slamType>
void Slam<slamType>::TrackMonocular(Frame &frame, Sophus::SE3f &tcw){
	if(slamType == eSlamType::ORBSLAM3){
		tcw = mpSlam->TrackMonocular(frame.getImage(), frame.getTimestampSec());
	}
}

template<eSlamType slamType>
void Slam<slamType>::InitialiseSlam(std::shared_ptr<StartupSlam::Request> request,
		std::shared_ptr<StartupSlam::Response> response){
}
