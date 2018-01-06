#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


void trackVelocityCallback(visualization_msgs::MarkerArray msg)
{
	cout<<"msg : "<<msg<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calculation_trajectries");
	ros::NodeHandle n;

	ros::Subscriber track_vel_sub = 
		n.subscribe("/kalman_filter/human_velocity", 1, trackVelocityCallback);

	cout<<"Here we go!!"<<endl;

	ros::spin();

	return 0;
}
