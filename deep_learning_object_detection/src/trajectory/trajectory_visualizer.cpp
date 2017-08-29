#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;

visualization_msgs::Marker trajectory;

ros::Publisher pub_trajectory;

void create_trajectory(CloudType::Ptr centroid, 
					   std_msgs::Header marker_header, 
					   visualization_msgs::Marker &marker)
{
	marker.header = marker_header;
	marker.ns = "trajectory";

	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.020;

	// marker.lifetime = ros::Duration(0.1);
	marker.lifetime = ros::Duration();

	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	geometry_msgs::Point p;
	p.x = centroid->points[0].x;
	p.y = centroid->points[0].y;
	p.z = centroid->points[0].z;

	if(p.z > -0.500){
		marker.points.push_back(p);
	}
}


void centroidCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	cout<<"==============================================="<<endl;
	CloudType::Ptr centroid (new CloudType);
	pcl::fromROSMsg(*msg, *centroid);
	for(size_t i=0; i<centroid->points.size(); i++){
		cout<<"centroid->points["<<i<<"].x : "<<centroid->points[i].x<<endl;
		cout<<"centroid->points["<<i<<"].y : "<<centroid->points[i].y<<endl;
		cout<<"centroid->points["<<i<<"].z : "<<centroid->points[i].z<<endl;
	}
	std_msgs::Header marker_header = msg->header;

	create_trajectory(centroid, marker_header, trajectory);

	pub_trajectory.publish(trajectory);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_visualizer");
	ros::NodeHandle n;

	pub_trajectory = n.advertise<visualization_msgs::Marker>("/human_points/centroid/trajectory", 1);

	ros::Subscriber sub_centroid = n.subscribe("/human_points/centroid", 1, centroidCallback);

	cout<<"Here we go!!"<<endl;

	ros::spin();

	return 0;
}

