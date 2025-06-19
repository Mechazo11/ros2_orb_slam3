![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-1.5.0-blue.svg)

# ROS2 ORB SLAM3 V1.0 package 

A ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. My goal is to provide a "bare-bones" starting point for developers in using ORB SLAM3 framework in their ROS 2 projects. Hence, this package will not use more advanced features of ROS 2 such as rviz, tf and launch files. This project structure is heavily influenced by the excellent ROS1 port of ORB SLAM3 by [thien94](https://github.com/thien94/orb_slam3_ros/tree/master). 

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

This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) `V1.0` as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]

`g2o` used packaged with `ORB-SLAM3` is a much older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o). If you are willing to make the forward port, please open a Ticket in this repository.

This package differs from other ROS1 wrappers, `thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV>= 4.2, 
  * Latest version of Pangolin
  * Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation

For newcomers into the ROS 2 ecosystem, this package serves as an example of `building a shared cpp library` and also `a package with both cpp and python nodes`.

* In **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano Orin, you need to extend `SWAP` space to at least `16Gb`. 

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

`LD_LIBRARY_PATH` is an environment variable that tells the dynamic linker where to search for shared libraries (.so files) at runtime. First we check if ```/usr/lib/local``` is available in `LD_LIBRARY_PATH`.

```bash
echo $LD_LIBRARY_PATH
```

If you see no output as shown below 

```bash
/opt/ros/jazzy/opt/gz_sim_vendor/lib:.....:/usr/lib/local
```

then `/usr/local/lib` is currently not in the `LD_LIBRARY_PATH` add it as shown below 

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
sudo ldconfig
```

##### Optional Step: Add /usr/local/lib permanently by modifying .bashrc

Open the ```.bashrc``` file in ```\home``` directory and add these lines at the very end

```bash
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```

Finally, source ```.bashrc``` file 

```bash
source ~/.bashrc
```
 
#### OpenCV
By default both Ubuntu 22.04 and Ubuntu 24.04 comes with >OpenCV 4.2. Check to make sure you have at least 4.2 installed. Run the following in a terminal

```bash
python3 -c "import cv2; print(cv2.__version__)" 
```

#### cv_brdige

Check to ensure `cv_bridge` package is installed in the `BASE WORKSPACE` i.e. `/ros/opt/<ROS-DISTRO>`. Try the following

```bash
ros2 pkg list | grep cv_bridge
```

If the string `cv_brdige` did not show up, install it as shown bellow

```bash
source /opt/ros/<ROS-DISTRO>/setup.bash
sudo apt-get install ros-<ROS-DISTRO>-cv-bridge 
```


## 2. Installation

Follow the steps below to create the ```ros2_test``` workspace, install dependencies and build the package. Note, the workspace must be named ```ros2_test``` due to a HARDCODED path in the python node. I leave it to the developers to change this behavior as they see fit.

```bash
cd ~
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
cd .. # make sure you are in ~/ros2_ws root directory
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

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
