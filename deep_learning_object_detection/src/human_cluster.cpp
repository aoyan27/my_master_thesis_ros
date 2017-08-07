#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

void humanPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *pointcloud);

	cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_cluster");
	ros::NodeHandle n;

	ros::Subscriber sub_humanpoints = n.subscribe("/human_points/debug", 1, humanPointsCallback);

	cout<<"Here we go!!!"<<endl;

	ros::spin();

	return 0;
}
