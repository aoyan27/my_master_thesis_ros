# my_master_thesis_ros
In this repository, I compiled the source code using ROS for the master thesis.

- camera_velodyne_calibration
	- camera_velodyne_calibration_gazebo
	- my_but_calibration_camera_velodyne
- deep_learning_object_recognition

## Hardware Spec
- PC
	- OS : Ubuntu 14.04 LTS
	- Memory : 32GB
	- CPU : Intel® Core™ i7-4790K CPU @ 4.00GHz × 8 
	- GPU : GeForce GTX TITAN X
	- Strage : 2TB

- Robot
	- Sensors
		- ZED(Stereolabs)
		- HDL-32e(Velodyne)
		- AMU-1802BR(IMU)
	- Vehcle
		- Differential drive

## Requirements
- ROS indigo(Ubuntu 14.04)
- [Chainer](https://github.com/pfnet/chainer) 2.0.0+
- Cython 0.25+ 
- OpenCV 2.4+, 3.1+
- CUDA 7.5+
- PCL 1.7+, 1.8+

## How to setup
### Install ROS indigo for Ubuntu 14.04
```
$ sudo apt-get install ros-indigo-desktop-full
```
**NOTE: Reference [Install ROS indigo](http://wiki.ros.org/ja/indigo/Installation/Ubuntu).**

### Setup sensors
- HDR-32e
```
$ cd $HOME
$ cd ros_catkin_ws/src
$ git clone https://github.com/ros-drivers/velodyne
$ cd ../
$ catkin_make
```

- ZED
	- Downlowd the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/release/2.1/#sdkdownloads_anchor) and setup the ZED SDK
**NOTE: Reference [ZED Documentation](https://www.stereolabs.com/documentation/overview/getting-started/introduction.html).**
	- Download zed-ros-wrapper
	```
	$ cd $HOME
	$ cd ros_catkin_ws/src
	$ git clone https://github.com/stereolabs/zed-ros-wrapper
	$ cd ../
	$ catkin_make
	```

## How to run
- Calibrate Camera and Velodyne

- Estimate human trajectory

