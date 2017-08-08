#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

ros::Publisher pub_debug;
ros::Publisher pub_debug2;

void plane_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
				  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,
				  pcl::PointIndices::Ptr inliers)
{
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(input_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*output_cloud);
	cout<<"PointCloud representing the planar component : "<<output_cloud->width * output_cloud->height<<" data points."<<endl;
}

void plane_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, 
						pcl::ModelCoefficients::Ptr coefficients,
						pcl::PointIndices::Ptr inliers)
{
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold(0.1);

	seg.setInputCloud(pointcloud);
	seg.segment(*inliers, *coefficients);

	if(inliers->indices.size() == 0){
		ROS_ERROR("Could not estimate a planar model for the given dataset.");
		return;
	}

	cout<<"Model coefficients : "<<coefficients->values[0]<<"\t"
								 <<coefficients->values[1]<<"\t"
								 <<coefficients->values[2]<<"\t"
								 <<coefficients->values[3]<<endl;
	cout<<"Model inliers : "<<inliers->indices.size()<<endl;
	size_t inliers_size = inliers->indices.size();
	for(size_t i = 0;i < inliers_size; i++){
		pointcloud->points[inliers->indices[i]].r = 255;
		pointcloud->points[inliers->indices[i]].g = 0;
		pointcloud->points[inliers->indices[i]].b = 0;
	}
}

void humanPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *pointcloud);
	cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	plane_segmentation(pointcloud, coefficients, inliers);

	sensor_msgs::PointCloud2 debug_pc;
	pcl::toROSMsg(*pointcloud, debug_pc);
	debug_pc.header = msg->header;
	pub_debug.publish(debug_pc);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	plane_removal(pointcloud, removed_points, inliers);

	sensor_msgs::PointCloud2 debug_pc2;
	pcl::toROSMsg(*removed_points, debug_pc2);
	debug_pc2.header = msg->header;
	pub_debug2.publish(debug_pc2);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_cluster");
	ros::NodeHandle n;

	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug", 1);
	pub_debug2 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug2", 1);

	ros::Subscriber sub_humanpoints = n.subscribe("/human_points/candidate", 1, humanPointsCallback);

	cout<<"Here we go!!!"<<endl;

	ros::spin();

	return 0;
}
