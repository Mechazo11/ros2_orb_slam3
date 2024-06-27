# ROS2 ORB SLAM3 V1.0 package

A ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. My goal is to provide a "bare-bones" starting point for developers in using ORB SLAM3 framework in their ROS 2 projects. Hence, this package will not use more advanced features of ROS 2 such as rviz, tf and launch files. This project structure is heavily influenced by the excellent ROS1 port of ORB SLAM3 by [thien94](https://github.com/thien94/orb_slam3_ros/tree/master). 

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
The following softwares must be installed before this package can be installed and used correctly

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
```
echo $LD_LIBRARY_PATH
```
If not, then perform the following 
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
sudo ldconfig
```
Then open the ```.bashrc``` file in ```\home``` directory and add these lines at the very end
```
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```
Finally, source ```.bashrc``` file 
```
source ~/.bashrc
```
 
### OpenCV
Ubuntu 22.04 by default comes with >OpenCV 4.2. Check to make sure you have at least 4.2 installed. Run the following in a terminal
```
python3 -c "import cv2; print(cv2.__version__)" 
```

## 2. Installation
1. In a new terminal move to home directory
```
cd ~
```
2. Create the ```ros2_test``` workspace, and download this package as shown below.
```
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
cd .. # make sure you are in ~/ros2_ws root directory
```
3. For this repo, the name of the workspace must be ```ros2_test```. You may change it later (marked with "!Change this ...." comments found in pertinent .hpp and .py files.

4. Source ROS2 Humble tools and run colcon build commands
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 3. Monocular Example:
Run the builtin example to verify the package is working correctly
In one terminal [cpp node]
```
cd ~\ros2_ws
source install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

In another terminal [python node]
```
cd ~\ros2_ws
source install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

Both nodes would perform a handshake and the VSLAM framework would then work as shown in the following video clip


https://github.com/Mechazo11/ros2_orb_slam3/assets/44814419/af9eaa79-da4b-4405-a4d7-e09242ab9660




Thank you for taking the time in checking this project out. I hope it helps you out. If you find this package useful in your project consider citing the original ORB SLAM3 paper and one of my recent papers shown below

## To-do:
- [x] Finish working example and upload code
- [x] Detailed installation and usage instructions
- [x] Show short video example for monocular mode
- [ ] Stereo mode example
- [ ] RGBD mode example
- [ ] Add ORB SLAM3 bibtex and my IEE AIM 2024 paper
