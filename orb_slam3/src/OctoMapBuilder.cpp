#include "OctoMapBuilder.h"

namespace ORB_SLAM3{
OctoMapBuilder::OctoMapBuilder(){
	std::cout<<"Creating OctoMapBuilder Interface"<<std::endl;
}

void OctoMapBuilder::SetFrameMapPointUpdateCallback(std::function<void(std::vector<MapPoint*>&, const Sophus::SE3<float>&)> frameUpdateCallback){
	mpFrameMapPointUpdateCallback = frameUpdateCallback;
}

void OctoMapBuilder::FrameMapPointUpdateCallback(std::vector<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
	mpFrameMapPointUpdateCallback(mapPoints, tcw);
}

void OctoMapBuilder::FrameMapPointUpdateCallback(std::set<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
	std::vector<MapPoint*> vMapPoints(mapPoints.begin(), mapPoints.end());
	mpFrameMapPointUpdateCallback(vMapPoints, tcw);
}
}
