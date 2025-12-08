![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-2.0.0-blue.svg)

# ROS2 ORB SLAM3 V1.0 package 

A ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. This is the `main/humble` branch which only supports ROS 2 Humble. Switch over to `jazzy` branch to use with ROS 2 Jazzy.

My goal is to provide a "bare-bones" starting point for developers in using ORB SLAM3 framework in their ROS 2 projects. Hence, this package will not use any advanced features of ROS 2 such as rviz, tf, launch files etc. The project structure is heavily influenced by the excellent ROS1 port of ORB SLAM3 by [thien94](https://github.com/thien94/orb_slam3_ros/tree/master). 

If you find this work useful please consider citing the original ORB-SLAM3 paper and my recent paper that uses this package in solving short-term relocalization (kidnapped robot problem) as shown below

```bibtex
@INPROCEEDINGS{kamal2024solving,
  author={Kamal, Azmyin Md. and Dadson, Nenyi Kweku Nkensen and Gegg, Donovan and Barbalata, Corina},
  booktitle={2024 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
  title={Solving Short-Term Relocalization Problems In Monocular Keyframe Visual SLAM Using Spatial And Semantic Data}, 
  year={2024},
  volume={},
  number={},
  pages={615-622},
  keywords={Visualization;Simultaneous localization and mapping;Accuracy;Three-dimensional displays;Semantics;Robot vision systems;Pipelines},
  doi={10.1109/AIM55361.2024.10637187}}
```

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 }
```

## 0. Preamble

* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]
* g2o used is an older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o).
* This package differs from other ROS1 wrappers, thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * At least C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV 4.2, latest release of Pangolin
* Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation
* For newcomers in ROS2 ecosystem, this package serves as an example of building a shared cpp library and also a package with both cpp and python nodes.
* May not build or work correctly in **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano

## Testing platforms

1. Intel i5-9300H, x86_64 bit architecture , Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)
2. AMD Ryzen 5600X, x86_64 bit architecture, Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)

## 1. Prerequisitis

Start with installing the following prerequisits

### Eigen3

```
sudo apt install libeigen3-dev
```

### Pangolin and configuring dynamic library path

We install Pangolin system wide and configure the dynamic library path so the necessary .so from Pangolin can be found by ros2 package during run time. More info here https://robotics.stackexchange.com/questions/105973/ros2-port-of-orb-slam3-can-copy-libdow2-so-and-libg2o-so-using-cmake-but-gettin

#### Install Pangolin

```
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended # Check what recommended softwares needs to be installed
./scripts/install_prerequisites.sh recommended # Install recommended dependencies
cmake -B build
cmake --build build -j4
sudo cmake --install build
```

#### Configure dynamic library

Check if ```/usr/lib/local``` is in the LIBRARY PATH

```bash
echo $LD_LIBRARY_PATH
```

If not, then perform the following 

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
sudo ldconfig
```

Then open the ```.bashrc``` file in ```\home``` directory and add these lines at the very end

```bash
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```

Finally, source ```.bashrc``` file 

```bash
source ~/.bashrc
```
 
### OpenCV
Ubuntu 22.04 by default comes with >OpenCV 4.2. Check to make sure you have at least 4.2 installed. Run the following in a terminal

```bash
python3 -c "import cv2; print(cv2.__version__)" 
```

## 2. Installation

Follow the steps below to create the ```ros2_ws``` workspace, install dependencies and build the package. Note, the workspace must be named ```ros2_ws``` due to a HARDCODED path in the python node. I leave it to the developers to change this behavior as they see fit.

```bash
cd ~
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
cd .. # make sure you are in ~/ros2_ws root directory
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### SBC Adjustments (Raspberry Pi, Jetson Orin Nano, etc)
The default branch of this repositoriy does not successfully compile for arm-based single-board computers like the Jetson Orin Nano. The arm-support branch makes adjustments so that this project can compile successfully. These details are for the Jetson Orin Nano on the Jetpack 6.2 SDK, adjustments may be needed for other boards.

#### Additional Packages
During testing, compilation failed a few times due to additional missing dependencies. For a Jetson Orin Nano on Ubuntu 22.04, install the following packages:

```
sudo apt install libdc1394-dev libopenexr-dev
```

In addition, ROS2 will automatically install opencv 4.8.0 which conflicts with version 4.5.4 installed by the Nvidia Jetpack SDK. This will cause versioning issues when trying to use things like CV bridge. To fix this, hold the opencv package at version 4.5.4:
```
sudo apt install libopencv-dev=4.5.4+dfsg-9ubuntu4
```

#### Compilation Considerations
I would run into OOM issues, so I had to [extend the size of SWAP](https://www.forecr.io/blogs/programming/how-to-increase-swap-space-on-jetson-modules) on the Orin to 16gb. In addition, I limited the number of jobs to 4 to prevent crashing during compilation. These considerations are accounted for in this build command:

```
colcon build --symlink-install --packages-select ros2_orb_slam3 --parallel-workers 4 --cmake-args -DCMAKE_CXX_FLAGS="-w" # hide warning so we know what fails
```

This will still take a while, so it's a good idea to run the process in a multiplexer like tmux or zellij so the process can be inspected later for issues. This build command also disables warnings so that errors are easier to find if the build fails, but it can be adjusted if necessary.

## 3. Monocular Example:

Run the builtin example to verify the package is working correctly
In one terminal [cpp node]

```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

In another terminal [python node]

```bash
cd ~/ros2_ws
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

Both nodes would perform a handshake and the VSLAM framework would then work as shown in the following video clip


https://github.com/Mechazo11/ros2_orb_slam3/assets/44814419/af9eaa79-da4b-4405-a4d7-e09242ab9660


Thank you for taking the time in checking this project out. I hope it helps you out. If you find this package useful in your project consider citing the papers mentioned above

## TODO next version:

- [ ] Stereo mode example
- [ ] RGBD mode example
- [ ] Detailed build instructions for `aarch64` based computers i.e. Orin Nano, Raspberry Pi etc.
