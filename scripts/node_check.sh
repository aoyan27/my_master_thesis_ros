#!/bin/bash
SCRIPT=/home/amsl/ros_catkin_ws/src/master_thesis/scripts/

str=(
	"/NormalEstimationForVelodyne"
	"/bounding_box_visualizer"
	"/cloud_nodelet"
	"/coloring"
	"/driver_nodelet"
	"/heightmap_node"
	"/human_cluster"
	"/human_points_extractor"
	"/image_republisher"
	"/my_sdd_with_ros"
	"/normal_vector_visualizer"
	"/rosout"
	"/rviz"
	"/trajectory_visualizer"
	"/velodyne_nodelet_manager"
)
echo ${str[@]}
rosnode list > "$SCRIPT"hoge.txt
while :
do
	rm "$SCRIPT"disp.sh
	for(( i=0; i<${#str[@]}; i++ ))
	do
		grep -q "${str[i]}" "$SCRIPT"hoge.txt
		ret=$?
		if test ${ret} -eq 0
		then
			echo "echo -e \"\e[36m ${str[i]} \e[m\"" >> "$SCRIPT"disp.sh
		else
			echo "echo -e \"\e[31m ${str[i]} \e[m\"" >> "$SCRIPT"disp.sh
		fi
	done

	clear

	source "$SCRIPT"disp.sh

	rosnode list > "$SCRIPT"hoge.txt
	sleep 0.1s
done
