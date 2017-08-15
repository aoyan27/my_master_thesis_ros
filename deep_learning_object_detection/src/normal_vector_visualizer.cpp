#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <math.h>

using namespace std;

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;

ros::Publisher pub_normal_arrows;

float arrow_scale = 0.1;

float vector_norm(float x, float y, float z)
{
	return pow((x * x + y * y + z * z), 0.5);
}

void calculate_orientation(PointType point, geometry_msgs::Quaternion &quat)
{
	float roll, pitch, yaw;
	float norm = vector_norm(point.normal_x, point.normal_y, point.normal_z);
	roll = acos(point.normal_x / norm);
	pitch = acos(point.normal_y / norm);
	yaw = acos(point.normal_z / norm);
	// cout<<"roll : "<<roll<<"\tpitch : "<<pitch<<"\tyaw : "<<yaw<<endl;

	quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	// cout<<"quat : "<<quat<<endl;
}

void create_normal_arrows(CloudType::Ptr input_cloud, std_msgs::Header marker_header, visualization_msgs::MarkerArray &marker_array)
{
	visualization_msgs::Marker marker;
	marker.header = marker_header;
	marker.ns = "normal_vectors";

	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 1.0 * arrow_scale;
	marker.scale.y = 0.1 * arrow_scale;
	marker.scale.z = 0.1 * arrow_scale;

	// marker.lifetime = ros::Duration(0.1);
	marker.lifetime = ros::Duration();

	cout<<"Now create marker!!"<<endl;	
	size_t input_cloud_size = input_cloud->points.size();
	for(size_t i = 0; i < input_cloud_size; i++){
		marker.id = i;	//id変えないと上書きされていくよ！！
		marker.pose.position.x = input_cloud->points[i].x;
		marker.pose.position.y = input_cloud->points[i].y;
		marker.pose.position.z = input_cloud->points[i].z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.w = 1.0;

		calculate_orientation(input_cloud->points[i], marker.pose.orientation);

		// if(i%3==0){
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
			marker.color.a = 1.0;
			marker_array.markers.push_back(marker);
		// }
	}
	cout<<"Finish!!"<<endl;	
	cout<<marker_array.markers.size()<<endl;
}

void perfectVelodyneNormalColoredCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	CloudType::Ptr pointcloud (new CloudType);
	pcl::fromROSMsg(*msg, *pointcloud);
	// cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;
	visualization_msgs::MarkerArray marker_array;
	std_msgs::Header marker_header = msg->header;

	create_normal_arrows(pointcloud, marker_header, marker_array);

	pub_normal_arrows.publish(marker_array);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "normal_vector_visualizer");
	ros::NodeHandle n;
	
	pub_normal_arrows = n.advertise<visualization_msgs::MarkerArray>("/normal_arrows", 1);

	// ros::Subscriber sub_perfect_velodyne = n.subscribe("/perfect_velodyne/normal/colored", 1, perfectVelodyneNormalColoredCallback);
	// ros::Subscriber sub_perfect_velodyne = n.subscribe("/perfect_velodyne/normal", 1, perfectVelodyneNormalColoredCallback);
	ros::Subscriber sub_perfect_velodyne = n.subscribe("/human_points/debug3", 1, perfectVelodyneNormalColoredCallback);

	cout<<"Here we go!!!"<<endl;

	ros::spin();

	return 0;
}
