#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;

ros::Publisher pub_debug;
ros::Publisher pub_centroid_cloud;


void coloring_minmax_point(CloudType::Ptr input_cloud, vector< vector<int> > minmax_index_list)
{
	for(size_t i = 0; i < minmax_index_list.size(); i++){
		for(size_t j = 0; j < minmax_index_list[i].size(); j++){
			if(i == 0){
				input_cloud->points[minmax_index_list[i][j]].r = 255;
				input_cloud->points[minmax_index_list[i][j]].g = 255;
				input_cloud->points[minmax_index_list[i][j]].b = 0;
			}
			else if(i == 1){
				input_cloud->points[minmax_index_list[i][j]].r = 0;
				input_cloud->points[minmax_index_list[i][j]].g = 255;
				input_cloud->points[minmax_index_list[i][j]].b = 255;
			}
		}
	}
}

void search_xyz_minmax(CloudType::Ptr input_cloud, vector< vector<float> > &minmax_list, vector< vector<int> > &minmax_index_list)
{
	minmax_list.resize(2);
	for(size_t i = 0; i < minmax_list.size(); i++){
		minmax_list[i].resize(3);
		minmax_list[i][0] = input_cloud->points[0].x;
		minmax_list[i][1] = input_cloud->points[0].y;
		minmax_list[i][2] = input_cloud->points[0].z;
	}
	minmax_index_list = vector< vector<int> >(2, vector<int>(3, 0));

	size_t input_cloud_size = input_cloud->points.size();
	for(size_t i = 1; i < input_cloud_size; i++){
		vector<float> xyz_point(3);
		xyz_point[0] = input_cloud->points[i].x;
		xyz_point[1] = input_cloud->points[i].y;
		xyz_point[2] = input_cloud->points[i].z;
		for(size_t j = 0; j < xyz_point.size(); j++){
			if(xyz_point[j] < minmax_list[0][j]){
				minmax_list[0][j] = xyz_point[j];
				minmax_index_list[0][j] = i;
			}
			if(xyz_point[j] > minmax_list[1][j]){
				minmax_list[1][j] = xyz_point[j];
				minmax_index_list[0][j] = i;
			}
		}
	}

	cout<<"i --> (0 : min, 1 : max)\tj --> (0 : x, 1 : y, 2 : y)"<<endl;
	for(size_t i = 0; i < minmax_list.size(); i++){
		for(size_t j = 0; j < minmax_list[i].size(); j++){
			cout<<"minmax_list["<<i<<"]["<<j<<"] : "<<minmax_list[i][j]<<endl;
		}
	}
}

void calculate_centroid(CloudType input_cloud, PointType *centroid_point)
{
	Eigen::Vector4f xyz_centroid(0.0, 0.0, 0.0, 0.0);
	pcl::compute3DCentroid(input_cloud, xyz_centroid);
	centroid_point->x = xyz_centroid[0];
	centroid_point->y = xyz_centroid[1];
	centroid_point->z = xyz_centroid[2];
}

void humanPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	CloudType::Ptr pointcloud (new CloudType);
	pcl::fromROSMsg(*msg, *pointcloud);
	cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;

	PointType centroid;
	calculate_centroid(*pointcloud, &centroid);
	// cout<<"centroid : "<<centroid<<endl;
	CloudType::Ptr centroid_cloud (new CloudType);
	centroid_cloud->points.push_back(centroid);
	
	sensor_msgs::PointCloud2 centroid_cloud_pc2;
	pcl::toROSMsg(*centroid_cloud, centroid_cloud_pc2);
	centroid_cloud_pc2.header = msg->header;
	pub_centroid_cloud.publish(centroid_cloud_pc2);

	vector< vector<float> > minmax_list;
	vector< vector<int> > minmax_index_list;
	search_xyz_minmax(pointcloud, minmax_list, minmax_index_list);

	coloring_minmax_point(pointcloud, minmax_index_list);

	sensor_msgs::PointCloud2 debug_pc;
	pcl::toROSMsg(*pointcloud, debug_pc);
	debug_pc.header = msg->header;
	pub_debug.publish(debug_pc);


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bounding_box_visualizer");
	ros::NodeHandle n;

	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/bbox/debug", 1);
	pub_centroid_cloud = n.advertise<sensor_msgs::PointCloud2>("/human_points/centroid", 1);

	ros::Subscriber sub_human_points = n.subscribe("/human_points", 1, humanPointsCallback);
	cout<<"Here we go!!"<<endl;

	ros::spin();
	return 0;
}
