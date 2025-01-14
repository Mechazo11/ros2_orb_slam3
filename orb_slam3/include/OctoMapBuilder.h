#ifndef OCTOMAPBUILDER_H
#define OCTOMAPBUILDER_H
#include <memory>
#include "MapPoint.h"

namespace ORB_SLAM3{
class OctoMapBuilder{
	public:
		OctoMapBuilder();
		void SetFrameMapPointUpdateCallback(std::function<void(std::vector<MapPoint*>&, const Sophus::SE3<float>&)> frameUpdateCallback);
		void SetGlobalMPAndKFPosesCallback(std::function<void(std::vector<std::pair<std::vector<MapPoint*>&, const Sophus::SE3<float>&>>)> globalMPAndKFPosesCallback);
		bool Initialise();
		void FrameMapPointUpdateCallback(std::vector<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		void FrameMapPointUpdateCallback(std::set<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		void GlobalMPAndKFPosesCallback(std::vector<std::pair<std::vector<MapPoint*>&, const Sophus::SE3<float>&>> &globalMPAndKFPoses);
	private:
		System* mpSystem;
		std::function<void(std::vector<MapPoint*>&, const Sophus::SE3<float>&)> mpFrameMapPointUpdateCallback;
		std::function<void(std::vector<std::pair<std::vector<MapPoint*>&, const Sophus::SE3<float>&>>)> mpGlobalMPAndKFPosesCallback;
};
}
#endif
