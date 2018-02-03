#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <time.h>

using namespace std;

class CvtMocapData
{
	private:
		ros::Subscriber mocap_velocity_sub_; 
		
		ros::Publisher my_agent_velocity_pub_;
		ros::Publisher other_agents_velocity_pub_;
		ros::Publisher my_agent_tracking_pub_;
		ros::Publisher other_agents_tracking_pub_;

		geometry_msgs::PoseStamped get_map_pose(geometry_msgs::PoseStamped mocap_pose);
		void set_marker(visualization_msgs::Marker &marker, 
					    int id, 
					    geometry_msgs::Vector3 scale, 
					    geometry_msgs::PoseStamped pose);

		void set_pointcloud(visualization_msgs::Marker marker);

		void mocapVelocityCallback(visualization_msgs::MarkerArray msg);

		visualization_msgs::Marker my_agent_;
		visualization_msgs::MarkerArray other_agents_;

		tf::TransformListener tflistener_;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr my_agent_tracking_pc;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr other_agents_tracking_pc;
	public:
		CvtMocapData(ros::NodeHandle &n);


};

CvtMocapData::CvtMocapData(ros::NodeHandle &n)
{
	mocap_velocity_sub_
		= n.subscribe("/velocity_arrows", 1, &CvtMocapData::mocapVelocityCallback, this);

	my_agent_velocity_pub_ = n.advertise<visualization_msgs::Marker>("/my_agent_velocity", 1);
	other_agents_velocity_pub_ = n.advertise<visualization_msgs::MarkerArray>("/other_agents_velocity", 1);
	my_agent_tracking_pub_ = n.advertise<sensor_msgs::PointCloud2>("/my_agent_velocity/tracking_point", 1);
	other_agents_tracking_pub_ = n.advertise<sensor_msgs::PointCloud2>("/other_agents_velocity/tracking_point", 1);

	my_agent_tracking_pc.reset (new pcl::PointCloud<pcl::PointXYZINormal>);
	other_agents_tracking_pc.reset (new pcl::PointCloud<pcl::PointXYZINormal>);
	
}


geometry_msgs::PoseStamped CvtMocapData::get_map_pose(geometry_msgs::PoseStamped mocap_pose)
{
	geometry_msgs::PoseStamped map_pose;
	map_pose.header.frame_id = "/input_map";
	map_pose.header.stamp = mocap_pose.header.stamp;
	try{
		tflistener_.waitForTransform("/input_map", "/world", 
									 map_pose.header.stamp, 
									 ros::Duration(5.0));
		tflistener_.transformPose("/input_map", map_pose.header.stamp, mocap_pose, 
								  "/world", map_pose);
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("getPose() : can't transform_listener : %s\n", ex.what());
	}

	return map_pose;
}

void CvtMocapData::set_marker(visualization_msgs::Marker &marker, 
						   	  int id, 
							  geometry_msgs::Vector3 scale, 
							  geometry_msgs::PoseStamped pose)
{
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = ros::Time::now();

	marker.id = id;

	if(id == 0){
		marker.ns = "my_agent";
	}
	else{
		ostringstream ss;
		ss << id;
		marker.ns = "human_" + ss.str();
	}


	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.020;

	marker.lifetime = ros::Duration(0.1);
	// marker.lifetime = ros::Duration();
	
	if(id == 0){
		marker.color.r = 228.0 / 255.0;
		marker.color.g = 162.0 / 255.0;
		marker.color.b = 11.0 / 255.0;
		marker.color.a = 1.0;
	}
	else{
		marker.color.r = 49.0 / 255.0;
		marker.color.g = 0.0 / 255.0;
		marker.color.b = 178.0 / 255.0;
		marker.color.a = 1.0;
	}

	marker.scale = scale;

	marker.pose = pose.pose;
}

void CvtMocapData::set_pointcloud(visualization_msgs::Marker marker)
{
	pcl::PointXYZINormal tmp_point;
	tmp_point.x = marker.pose.position.x;
	tmp_point.y = marker.pose.position.y;
	tmp_point.z = marker.pose.position.z;
	tmp_point.curvature = marker.id;

	if(marker.id == 0){
		my_agent_tracking_pc->points.clear();
		my_agent_tracking_pc->points.push_back(tmp_point);
		// cout<<my_agent_tracking_pc->points.size()<<endl;
	}
	else{
		other_agents_tracking_pc->points.clear();
		other_agents_tracking_pc->points.push_back(tmp_point);
	}
}


void CvtMocapData::mocapVelocityCallback(visualization_msgs::MarkerArray msg)
{
	other_agents_.markers.clear();
	size_t num_agents = msg.markers.size();
	cout<<"num_agents : "<<num_agents<<endl;
	clock_t start = clock();
	for(size_t i=0;i<num_agents;i++){
		// cout<<"========================="<<endl;
		geometry_msgs::PoseStamped tmp_mocap;
		tmp_mocap.header = msg.markers[i].header;
		tmp_mocap.pose = msg.markers[i].pose;
		// cout<<"tmp_mocap : "<<tmp_mocap<<endl;
		// double tmp_mocap_yaw = tf::getYaw(tmp_mocap.pose.orientation);
		// cout<<"tmp_mocap_yaw : "<<tmp_mocap_yaw<<endl;

		geometry_msgs::PoseStamped tmp_map;
		tmp_map = get_map_pose(tmp_mocap);
		// cout<<"tmp_map : "<<tmp_map<<endl;
		// double tmp_map_yaw = tf::getYaw(tmp_mocap.pose.orientation);
		// cout<<"tmp_map_yaw : "<<tmp_map_yaw<<endl;
		if(i == 0){
			set_marker(my_agent_, msg.markers[i].id, msg.markers[i].scale, tmp_map);
			set_pointcloud(my_agent_);
		}
		else{
			visualization_msgs::Marker tmp_marker;
			set_marker(tmp_marker, msg.markers[i].id, msg.markers[i].scale, tmp_map);
			other_agents_.markers.push_back(tmp_marker);
			set_pointcloud(tmp_marker);
		}

	}

	my_agent_velocity_pub_.publish(my_agent_);
	other_agents_velocity_pub_.publish(other_agents_);

	std_msgs::Header header;
	header.frame_id = "/input_map";
	header.stamp = ros::Time::now();

	sensor_msgs::PointCloud2 my_agent_tracking_pc2;
	pcl::toROSMsg(*my_agent_tracking_pc, my_agent_tracking_pc2);
	my_agent_tracking_pc2.header = header;
	my_agent_tracking_pub_.publish(my_agent_tracking_pc2);

	sensor_msgs::PointCloud2 other_agents_tracking_pc2;
	pcl::toROSMsg(*other_agents_tracking_pc, other_agents_tracking_pc2);
	other_agents_tracking_pc2.header = header;
	other_agents_tracking_pub_.publish(other_agents_tracking_pc2);



	clock_t end = clock();
	cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "convert_motion_captuer_world2map");
	ros::NodeHandle n;

	CvtMocapData cmd(n);

	cout<<"Here we go!!"<<endl;

	ros::spin();

	return 0;
}
