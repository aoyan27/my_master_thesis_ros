#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

gnome-terminal -e "/opt/ros/indigo/bin/roscore" --geometry=50x12+0+0 &
sleep 3s

gnome-terminal -e "/opt/ros/indigo/bin/roslaunch velodyne_pointcloud 32e_points.launch" --geometry=50x12+0+250 &
gnome-terminal -e "/opt/ros/indigo/bin/rosrun image_transport republish compressed in:=/zed/rgb/image_raw_color raw out:=/zed/rgb/image_raw_color" --geometry=50x12+0+500 &


gnome-terminal -e "/opt/ros/indigo/bin/roslaunch deep_learning_object_detection normal_estimation_colored.launch" --geometry=50x12+0+750 &
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch my_but_calibration_camera_velodyne coloring.launch" --geometry=50x12+500+0 &
gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection heightmap_node" --geometry=50x12+500+250 &


gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection my_ssd_with_ros.py --gpu 0" --geometry=50x12+500+500 &


gnome-terminal -e "/opt/ros/indigo/bin/roslaunch deep_learning_object_detection human_extract.launch" --geometry=50x12+500+750 &
gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection human_cluster" --geometry=50x12+1000+0 &


gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection normal_vector_visualizer" --geometry=50x12+1000+250 &
gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection bounding_box_visualizer" --geometry=50x12+1000+500 &
gnome-terminal -e "/opt/ros/indigo/bin/rosrun deep_learning_object_detection trajectory_visualizer" --geometry=50x12+1000+750 &


#### rviz  ####
gnome-terminal -e "/opt/ros/indigo/bin/rosrun rviz rviz -d /home/amsl/.rviz/velodyne_zed_for_master_thesis.rviz" --geometry=1x1+0+0 &


#### check whether nodes are running ####
gnome-terminal -e "/home/amsl/ros_catkin_ws/src/master_thesis/scripts/node_check.sh" --geometry=50x48+1500+0
