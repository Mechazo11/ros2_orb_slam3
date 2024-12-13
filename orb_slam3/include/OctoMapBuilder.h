#ifndef OCTOMAPBUILDER_H
#define OCTOMAPBUILDER_H
#include <memory>
#include "MapPoint.h"

namespace ORB_SLAM3{
class OctoMapBuilder{
	public:
		OctoMapBuilder();
		void SetFrameMapPointUpdateCallback(std::function<void(std::vector<MapPoint*>&, const Sophus::SE3<float>&)> frameUpdateCallback);
		bool Initialise();
		void FrameMapPointUpdateCallback(std::vector<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		void FrameMapPointUpdateCallback(std::set<MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
	private:
		System* mpSystem;
		std::function<void(std::vector<MapPoint*>&, const Sophus::SE3<float>&)> mpFrameMapPointUpdateCallback;
};
}
#endif
