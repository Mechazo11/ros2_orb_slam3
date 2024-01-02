# ROS2 ORB SLAM3 V1.0 package

A native ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. My goal is to provide a "bare-bones" starting point for developers in using ORB SLAM3 framework in their ROS 2 projects. Hence, this package will not use more advanced features of ROS 2 such as rviz, tf and lauch files. This project structure is heavily influenced by the excellent ROS1 port of ORB SLAM3 by thien94, https://github.com/thien94/orb_slam3_ros/tree/master

## 0. Preamble
* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]
* g2o used is a 
* This package differs from other ROS1 wrappers, thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is a purely design choice.
  * At least C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV 4.2, latest release of Pangolin
* Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation

## 1. Prerequisitis

### Eigen3

```
sudo apt install libeigen3-dev
```

 

### OpenCV


## 2. Installation
Add pangolin installation instructions here

## 3. Monocular Example




## To-do:
- [x] Finish working example and upload code
- [ ] Detailed installation and usage instructions
- [ ] Show short video example for monocular mode
- [ ] Add delight to the experience when all tasks are complete :tada:
