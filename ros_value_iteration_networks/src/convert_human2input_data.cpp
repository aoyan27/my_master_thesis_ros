#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <time.h>


using namespace std;


#define NUM_EXTRACT 1

#define CONSTRAINT_X 5.0
#define CONSTRAINT_Y 5.0


class CvtHuman2Input
{
	private:
		ros::Subscriber human_velocity_sub_;
		ros::Publisher extract_human_pub_;


		void humanVelocityCallback(visualization_msgs::MarkerArray msg);
		geometry_msgs::PoseStamped get_local_pose(geometry_msgs::PoseStamped global_pose);
		double get_dist(geometry_msgs::Point a, geometry_msgs::Point b);
		void extract_input_humans(vector<int> human_id_list,
								  vector<geometry_msgs::Vector3> human_velocity_list,
								  vector<geometry_msgs::PoseStamped> human_pose_list, 
								  vector<double> dist_list, int num_extract);
		void set_marker(visualization_msgs::Marker &marker, 
						int id, 
						geometry_msgs::Vector3 scale, 
						geometry_msgs::PoseStamped pose);

		tf::TransformListener tflistener_;
		geometry_msgs::Point zero_point;
		visualization_msgs::MarkerArray extract_human;

	public:
		CvtHuman2Input(ros::NodeHandle &n);

};

CvtHuman2Input::CvtHuman2Input(ros::NodeHandle &n)
{
	human_velocity_sub_ 
		= n.subscribe("/velocity_arrows", 1, &CvtHuman2Input::humanVelocityCallback, this);

	extract_human_pub_ = n.advertise<visualization_msgs::MarkerArray>("/extract_human", 1);

	zero_point.x = 0.0;
	zero_point.y = 0.0;
	zero_point.z = 0.0;
}

geometry_msgs::PoseStamped
CvtHuman2Input::get_local_pose(geometry_msgs::PoseStamped global_pose)
{
	geometry_msgs::PoseStamped local_pose;
	local_pose.header.frame_id = "/velodyne";
	local_pose.header.stamp = global_pose.header.stamp;
	try{
		tflistener_.waitForTransform("/velodyne", "/map", 
									 local_pose.header.stamp, 
									 ros::Duration(5.0));
		tflistener_.transformPose("/velodyne", local_pose.header.stamp, global_pose, 
								  "/map", local_pose);
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("getPose() : can't transform_listener : %s\n", ex.what());
	}

	return local_pose;
}

double CvtHuman2Input::get_dist(geometry_msgs::Point a, geometry_msgs::Point b)
{
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

void CvtHuman2Input::set_marker(visualization_msgs::Marker &marker, 
								int id, 
								geometry_msgs::Vector3 scale, 
								geometry_msgs::PoseStamped pose)
{
	marker.header.frame_id = "/velodyne";
	marker.header.stamp = ros::Time::now();

	marker.id = id;

	ostringstream ss;
	ss << id;
	marker.ns = "human_" + ss.str();


	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.020;

	marker.lifetime = ros::Duration(0.1);
	// marker.lifetime = ros::Duration();

	marker.color.r = 228.0 / 255.0;
	marker.color.g = 162.0 / 255.0;
	marker.color.b = 11.0 / 255.0;
	marker.color.a = 1.0;

	marker.scale = scale;

	marker.pose = pose.pose;
}
	
void CvtHuman2Input::extract_input_humans(vector<int> human_id_list, 
										  vector<geometry_msgs::Vector3> human_velocity_list,  
										  vector<geometry_msgs::PoseStamped> human_pose_list, 
										  vector<double> dist_list, int num_extract)
{
	cout<<"num_extract : "<<num_extract<<endl;
	vector<double> tmp_dist_list = dist_list;
	sort(tmp_dist_list.begin(), tmp_dist_list.end(), greater<double>());
	for(int i=0; i<num_extract; i++){
		visualization_msgs::Marker marker;
		double min_dist = tmp_dist_list[tmp_dist_list.size()-1];
		// cout<<"min_dist : "<<min_dist<<endl;
		auto min_iter = find(dist_list.begin(), dist_list.end(), min_dist);
		size_t min_index = distance(dist_list.begin(), min_iter);
		// cout<<"min_index : "<<min_index<<endl;
		tmp_dist_list.pop_back();

		set_marker(marker, 
				   human_id_list[min_index], 
				   human_velocity_list[min_index], 
				   human_pose_list[min_index]);

		extract_human.markers.push_back(marker);
	}

	extract_human_pub_.publish(extract_human);
}


void CvtHuman2Input::humanVelocityCallback(visualization_msgs::MarkerArray msg)
{
	cout<<"===================================="<<endl;
	extract_human.markers.clear();

	size_t num_human = msg.markers.size();
	cout<<"num_human : "<<num_human<<endl;
	if(num_human != 0){
		vector<int> human_id_list;
		vector<geometry_msgs::Vector3> human_velocity_list;
		vector<geometry_msgs::PoseStamped> human_pose_list;
		vector<double> dist_list;
		
		clock_t start = clock();
		for(size_t i=0; i<num_human; i++){
			// cout<<"human_id : "<<msg.markers[i].id<<endl;
			human_id_list.push_back(msg.markers[i].id);
			human_velocity_list.push_back(msg.markers[i].scale);

			geometry_msgs::PoseStamped tmp_global;
			tmp_global.header = msg.markers[i].header;
			tmp_global.pose = msg.markers[i].pose;
			// cout<<"tmp_global.pose.position : "<<tmp_global.pose.position<<endl;
			// double tmp_global_yaw = tf::getYaw(tmp_global.pose.orientation);
			// cout<<"tmp_global_yaw : "<<tmp_global_yaw<<endl;

			geometry_msgs::PoseStamped tmp_local;
			tmp_local = CvtHuman2Input::get_local_pose(tmp_global);
			// cout<<"tmp_lcoal.pose.position : "<<tmp_local.pose.position<<endl;
			// double tmp_local_yaw = tf::getYaw(tmp_local.pose.orientation);
			// cout<<"tmp_local_yaw : "<<tmp_local_yaw<<endl;
			if(fabs(tmp_local.pose.position.x) <= CONSTRAINT_X 
					&& fabs(tmp_local.pose.position.y) <= CONSTRAINT_Y)
			{
				human_pose_list.push_back(tmp_local);
				double dist = get_dist(tmp_local.pose.position, zero_point);
				dist_list.push_back(dist);
			}
		}

		size_t num_human_in_range = human_pose_list.size();
		cout<<"num_human_in_range : "<<num_human_in_range<<endl;
		if(num_human_in_range >= NUM_EXTRACT){
			extract_input_humans(human_id_list, human_velocity_list, human_pose_list, 
								 dist_list, NUM_EXTRACT);
		}
		else{
			extract_input_humans(human_id_list, human_velocity_list, human_pose_list, 
								 dist_list, num_human_in_range);
		}
		clock_t end = clock();
		cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "convert_human2input_data");
	ros::NodeHandle n;

	CvtHuman2Input chi(n);

	cout<<"Here we go!!"<<endl;

	ros::spin();

	return 0;

}
