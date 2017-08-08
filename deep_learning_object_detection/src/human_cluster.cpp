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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

// typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;

ros::Publisher pub_debug;
ros::Publisher pub_debug2;
ros::Publisher pub_debug3;

void cluster(CloudType::Ptr input_cloud,
			 CloudType::Ptr cluster_cloud,
			 std::vector<pcl::PointIndices> &cluster_indices)
{
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	tree->setInputCloud(input_cloud);

	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(0.500);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input_cloud);
	ec.extract(cluster_indices);

	// std::vector<CloudType> clusters;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cluster_cloud->points.push_back(input_cloud->points[*pit]);
		}
		cout<<"cluster_cloud->points.size() : "<<cluster_cloud->points.size()<<endl;
	}
}

void plane_removal(CloudType::Ptr input_cloud,
				  CloudType::Ptr output_cloud,
				  pcl::PointIndices::Ptr inliers)
{
	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(input_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*output_cloud);
	cout<<"PointCloud representing the planar component : "
		<<output_cloud->width * output_cloud->height<<" data points."<<endl;
}

void plane_segmentation(CloudType::Ptr pointcloud, 
						pcl::ModelCoefficients::Ptr coefficients,
						pcl::PointIndices::Ptr inliers)
{
	pcl::SACSegmentation<PointType> seg;
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
	CloudType::Ptr pointcloud (new CloudType);
	pcl::fromROSMsg(*msg, *pointcloud);
	cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	CloudType::Ptr removed_points (new CloudType);

	plane_segmentation(pointcloud, coefficients, inliers);
	plane_removal(pointcloud, removed_points, inliers);

	std::vector<pcl::PointIndices> cluster_indices;
	CloudType::Ptr cluster_cloud (new CloudType);
	cluster(removed_points, cluster_cloud, cluster_indices);

	cout<<"cluster_indices.size() : "<<cluster_indices.size()<<endl;


	sensor_msgs::PointCloud2 debug_pc;
	pcl::toROSMsg(*pointcloud, debug_pc);
	debug_pc.header = msg->header;
	pub_debug.publish(debug_pc);

	sensor_msgs::PointCloud2 debug_pc2;
	pcl::toROSMsg(*removed_points, debug_pc2);
	debug_pc2.header = msg->header;
	pub_debug2.publish(debug_pc2);

	sensor_msgs::PointCloud2 debug_pc3;
	pcl::toROSMsg(*cluster_cloud, debug_pc3);
	debug_pc3.header = msg->header;
	pub_debug3.publish(debug_pc3);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_cluster");
	ros::NodeHandle n;

	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug", 1);
	pub_debug2 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug2", 1);
	pub_debug3 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug3", 1);

	ros::Subscriber sub_humanpoints = n.subscribe("/human_points/candidate", 1, humanPointsCallback);

	cout<<"Here we go!!!"<<endl;

	ros::spin();

	return 0;
}
