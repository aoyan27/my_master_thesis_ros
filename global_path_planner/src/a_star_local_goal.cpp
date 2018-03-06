#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <time.h>

#ifdef _OEPNMP
#include <omp.h>
#endif

using namespace std;

nav_msgs::Path global_path;
nav_msgs::Odometry lcl;
nav_msgs::OccupancyGrid local_map;


bool sub_global_path = false;
bool sub_lcl = false;
bool sub_local_map = false;


float calc_dist_robo_path(geometry_msgs::PoseStamped path_pose)
{
	float robo_x = lcl.pose.pose.position.x;
	float robo_y = lcl.pose.pose.position.y;
	float path_x = path_pose.pose.position.x;
	float path_y = path_pose.pose.position.y;

	float dist = sqrt(pow(path_x-robo_x, 2.0) + pow(path_y-robo_y, 2.0));
	
	return dist;
}

geometry_msgs::Quaternion get_local_goal_orientation(geometry_msgs::PoseStamped current_pose, 
													geometry_msgs::PoseStamped next_pose)
// geometry_msgs::Quaternion get_local_goal_orientation(geometry_msgs::Pose current_pose, 
													// geometry_msgs::PoseStamped next_pose)
{
	// cout<<"current_pose : "<<current_pose<<endl;
	// cout<<"next_pose : "<<next_pose<<endl;
	double yaw = atan2(next_pose.pose.position.y-current_pose.pose.position.y, 
					   next_pose.pose.position.x-current_pose.pose.position.x);
	// double yaw = atan2(next_pose.pose.position.y-current_pose.position.y, 
					   // next_pose.pose.position.x-current_pose.position.x);
	// cout<<"yaw : "<<yaw<<endl;
	
	geometry_msgs::Quaternion q;
	q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
	return q;
}

void globalPathCallback(nav_msgs::Path msg)
{
	sub_global_path = true;

	global_path = msg;
	// cout<<"subscribe global_path!!"<<endl;
}

void localMapCallback(nav_msgs::OccupancyGrid msg)
{
	sub_local_map = true;

	local_map = msg;
	// cout<<"Subscribe local_map!!"<<endl;
}

void lclCallback(nav_msgs::Odometry msg)
{
	sub_lcl = true;

	lcl = msg;
	// cout<<"Subscribe lcl!!"<<endl;
}

void set_vis_local_goal(geometry_msgs::PoseStamped local_goal, 
						visualization_msgs::Marker &marker)
{
	marker.header = local_goal.header;
	marker.ns = "local_goal";

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;

	// marker.lifetime = ros::Duration(0.1);
	marker.lifetime = ros::Duration();

	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.pose = local_goal.pose;
}

geometry_msgs::PoseStamped get_local_goal(nav_msgs::Path global_path_)
{
	geometry_msgs::PoseStamped local_goal;
	if(sub_lcl && sub_local_map && sub_global_path){
		float resolution = local_map.info.resolution;
		float height = local_map.info.height * resolution;

		float goal_radius = height / 2.0 - resolution;
		// cout<<"goal_radius : "<<goal_radius<<endl;
		size_t global_path_length = global_path_.poses.size();
		if(global_path_length > 0){
			float min_diff_dist = fabs(calc_dist_robo_path(global_path_.poses[0]) - goal_radius);
			size_t min_index = 0;
			for(size_t i=1; i<global_path_length; i++){
				float tmp_min_diff_dist 
					= fabs(calc_dist_robo_path(global_path_.poses[i]) - goal_radius);
				if(tmp_min_diff_dist < min_diff_dist)
				{
					min_diff_dist = tmp_min_diff_dist;
					min_index = i;
				}
			}
			// cout<<"min_diff_dist : "<<min_diff_dist<<endl;
			// cout<<"min_index : "<<min_index<<endl;
			local_goal = global_path_.poses[min_index];
			// local_goal.pose.orientation = lcl.pose.pose.orientation;

			// local_goal.pose.orientation 
				// = get_local_goal_orientation(lcl.pose.pose, 
											 // global_path.poses[min_index]);
			if(global_path_length > 2 && min_index != 0)
			{
				local_goal.pose.orientation 
					= get_local_goal_orientation(global_path_.poses[min_index-1], 
												 global_path_.poses[min_index]);
			}
			printf("local_goal : (%.3f, %.3f, %.3f)\n", local_goal.pose.position.x, 
														local_goal.pose.position.y, 
														local_goal.pose.position.z);
		}
	}
	return local_goal;
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "a_star_local_goal");
	ros::NodeHandle n;

	ros::Subscriber local_map_sub = n.subscribe("/local_map_real", 1, localMapCallback);
	ros::Subscriber global_path_sub = n.subscribe("/global_path", 1, globalPathCallback);
	ros::Subscriber lcl_sub = n.subscribe("/lcl5", 1, lclCallback);


	ros::Publisher local_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
	ros::Publisher vis_local_goal_pub 
		= n.advertise<visualization_msgs::Marker>("/local_goal/vis", 1);


	cout<<"Here we go!!"<<endl;

	ros::Rate loop_rate(20);

	geometry_msgs::PoseStamped local_goal;
	visualization_msgs::Marker vis_local_goal;

	while(ros::ok()){
		if(sub_global_path && sub_local_map && sub_lcl){
			printf("=================================\n");
			local_goal = get_local_goal(global_path);
			local_goal.header.stamp = ros::Time::now();
			local_goal.header.frame_id = global_path.header.frame_id;

			set_vis_local_goal(local_goal, vis_local_goal);

			local_goal_pub.publish(local_goal);
			vis_local_goal_pub.publish(vis_local_goal);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
