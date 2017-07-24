#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

ros::Publisher pub;

sensor_msgs::PointCloud2 pc2;

void velodyne_callback(sensor_msgs::PointCloud msg){
	sensor_msgs::convertPointCloudToPointCloud2(msg, pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(pc2, *pcl_pc);
	
	cout<<"pcl_pc->points.size() : "<<pcl_pc->points.size()<<endl;

	// pub.publish(pc2);
	// cout<<"ASDSDASDA"<<endl;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "convert_pointcloud2");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/velodyne_points/points", 1, velodyne_callback);

	pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
	
	ros::Rate loop_rate(20);
	while(ros::ok()){
		pub.publish(pc2);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
