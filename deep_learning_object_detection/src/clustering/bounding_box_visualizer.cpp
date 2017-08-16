#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
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

ros::Publisher pub_min_points;
ros::Publisher pub_max_points;
ros::Publisher pub_bbox;

void create_bbox_marker(vector< vector<float> > minmax_list, 
						std_msgs::Header marker_header, 
						visualization_msgs::Marker &marker)
{
	marker.header = marker_header;
	marker.ns = "bbox";
	
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.025;

	marker.lifetime = ros::Duration(0.1);
	// marker.lifetime = ros::Duration();

	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	// set bbox vertex
	vector<geometry_msgs::Point> plane_vertex_list(4);

	geometry_msgs::Point p1;
	p1.x = minmax_list[0][0];
	p1.y = minmax_list[0][1];
	p1.z = minmax_list[0][2];
	plane_vertex_list[0] = p1;

	geometry_msgs::Point p2;
	p2.x = minmax_list[0][0];
	p2.y = minmax_list[1][1];
	p2.z = minmax_list[0][2];
	plane_vertex_list[1] = p2;

	geometry_msgs::Point p3;
	p3.x = minmax_list[1][0];
	p3.y = minmax_list[1][1];
	p3.z = minmax_list[0][2];
	plane_vertex_list[2] = p3;

	geometry_msgs::Point p4;
	p4.x = minmax_list[1][0];
	p4.y = minmax_list[0][1];
	p4.z = minmax_list[0][2];
	plane_vertex_list[3] = p4;

	vector< vector<geometry_msgs::Point> > vertex_list(2, plane_vertex_list);
	for(size_t i = 0; i < vertex_list[1].size(); i++){
		vertex_list[1][i].z = minmax_list[1][2];
	}

	// create xy-plane at minimum z and maximum z
	for(size_t i = 0; i < vertex_list.size(); i++){
		for(size_t j = 0; j < vertex_list[i].size(); j++){
			marker.points.push_back(vertex_list[i][j]);
			// cout<<"j : "<<j<<endl;
			size_t k = j + 1;
			if(k >= vertex_list[i].size()){
				k = 0;
			}
			// cout<<"j + 1 : "<<k<<endl;
			marker.points.push_back(vertex_list[i][k]);
		}
	}

	// connect min and max xy-plane 
	for(size_t i = 0; i < vertex_list[0].size(); i++){
		marker.points.push_back(vertex_list[0][i]);
		marker.points.push_back(vertex_list[1][i]);
	}
}

void coloring_minmax_point(CloudType::Ptr input_cloud, 
						   CloudType::Ptr min_points, 
						   CloudType::Ptr max_points, 
						   vector< vector<int> > minmax_index_list)
{
	for(size_t i = 0; i < minmax_index_list.size(); i++){
		for(size_t j = 0; j < minmax_index_list[i].size(); j++){
			PointType tmp;
			tmp = input_cloud->points[minmax_index_list[i][j]];
			if(i == 0){
				tmp.r = 255;
				tmp.g = 255;
				tmp.b = 0;
				min_points->points.push_back(tmp);
			}
			else if(i == 1){
				tmp.r = 0;
				tmp.g = 255;
				tmp.b = 255;
				max_points->points.push_back(tmp);
			}
		}
	}
}

void search_xyz_minmax(CloudType::Ptr input_cloud, 
					   vector< vector<float> > &minmax_list, 
					   vector< vector<int> > &minmax_index_list)
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
				minmax_index_list[1][j] = i;
			}
		}
	}

	// cout<<"i --> (0 : min, 1 : max)\tj --> (0 : x, 1 : y, 2 : y)"<<endl;
	// for(size_t i = 0; i < minmax_list.size(); i++){
		// for(size_t j = 0; j < minmax_list[i].size(); j++){
			// cout<<"minmax_list["<<i<<"]["<<j<<"] : "<<minmax_list[i][j]<<endl;
		// }
	// }
	// for(size_t i = 0; i < minmax_index_list.size(); i++){
		// for(size_t j = 0; j < minmax_index_list[i].size(); j++){
			// cout<<"minmax_index_list["<<i<<"]["<<j<<"] : "<<minmax_index_list[i][j]<<endl;
		// }
	// }
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

	if(pointcloud->points.size() == 0){
		ROS_ERROR("msg is empty.");
		return;
	}

	vector< vector<float> > minmax_list;
	vector< vector<int> > minmax_index_list;
	search_xyz_minmax(pointcloud, minmax_list, minmax_index_list);

	CloudType::Ptr min_points (new CloudType);
	CloudType::Ptr max_points (new CloudType);
	coloring_minmax_point(pointcloud, min_points, max_points, minmax_index_list);

	sensor_msgs::PointCloud2 min_points_pc2;
	pcl::toROSMsg(*min_points, min_points_pc2);
	min_points_pc2.header = msg->header;
	pub_min_points.publish(min_points_pc2);

	sensor_msgs::PointCloud2 max_points_pc2;
	pcl::toROSMsg(*max_points, max_points_pc2);
	max_points_pc2.header = msg->header;
	pub_max_points.publish(max_points_pc2);

	std_msgs::Header marker_header = msg->header;
	visualization_msgs::Marker bbox_marker;

	create_bbox_marker(minmax_list, marker_header, bbox_marker);
	pub_bbox.publish(bbox_marker);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bounding_box_visualizer");
	ros::NodeHandle n;

	pub_min_points = n.advertise<sensor_msgs::PointCloud2>("/bbox/min_points", 1);
	pub_max_points = n.advertise<sensor_msgs::PointCloud2>("/bbox/max_points", 1);
	pub_bbox = n.advertise<visualization_msgs::Marker>("/bbox/marker", 1);

	ros::Subscriber sub_human_points = n.subscribe("/human_points", 1, humanPointsCallback);
	cout<<"Here we go!!"<<endl;

	ros::spin();
	return 0;
}
