#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
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
ros::Publisher pub_debug4;
ros::Publisher pub_human_points;
ros::Publisher pub_centroid_cloud;

float calculate_distance_xy_plane(PointType input_point)
{
	return sqrt(pow(input_point.x, 2.0) + pow(input_point.y, 2.0));
}

void calculate_centroid(CloudType input_cloud, PointType *centroid_point)
{
	Eigen::Vector4f xyz_centroid(0.0, 0.0, 0.0, 0.0);
	pcl::compute3DCentroid(input_cloud, xyz_centroid);
	centroid_point->x = xyz_centroid[0];
	centroid_point->y = xyz_centroid[1];
	centroid_point->z = xyz_centroid[2];
}

void coloring_cluster(std::vector<CloudType> &cluster_list)
{
	size_t cluster_list_size = cluster_list.size();
	for(size_t i = 0; i < cluster_list_size; i++){
		size_t cluster_cloud_size = cluster_list[i].points.size();
		// cout<<"i : "<<i<<endl;
		for(size_t j = 0; j < cluster_cloud_size; j++){
			if(i == 0){
				// cout<<"red"<<endl;
				cluster_list[i].points[j].r = 255;
				cluster_list[i].points[j].g = 0;
				cluster_list[i].points[j].b = 0;
			}
			else if(i == 1){
				// cout<<"green"<<endl;
				cluster_list[i].points[j].r = 0;
				cluster_list[i].points[j].g = 255;
				cluster_list[i].points[j].b = 0;
			}
			else if(i == 2){
				// cout<<"bule"<<endl;
				cluster_list[i].points[j].r = 0;
				cluster_list[i].points[j].g = 0;
				cluster_list[i].points[j].b = 255;
			}
			else if(i == 3){
				// cout<<"black"<<endl;
				cluster_list[i].points[j].r = 0;
				cluster_list[i].points[j].g = 0;
				cluster_list[i].points[j].b = 0;
			}
			else{
				// cout<<"white"<<endl;
				cluster_list[i].points[j].r = 255;
				cluster_list[i].points[j].g = 255;
				cluster_list[i].points[j].b = 255;
			}
		}
	}
}

void check_cluster_min_distance(std::vector<CloudType> cluster_list, 
								CloudType::Ptr output_cloud)
{
	size_t cluster_list_size = cluster_list.size();
	if(cluster_list_size > 0){
		std::vector<PointType> centroid_list;
		centroid_list.resize(cluster_list_size);
		for(size_t i = 0; i < cluster_list_size; i++){
			PointType centroid_point;
			calculate_centroid(cluster_list[i], &centroid_point);
			cout<<"centroid_point : "<<centroid_point<<endl;
			centroid_list[i] = centroid_point;
		}

		float min_dist = calculate_distance_xy_plane(centroid_list[0]);
		int min_index = 0;
		size_t centroid_list_size = centroid_list.size();
		for(size_t i = 1; i < centroid_list_size; i++){
			float tmp_dist = calculate_distance_xy_plane(centroid_list[i]);
			if(tmp_dist < min_dist){
				min_dist = tmp_dist;
				min_index = i;
			}
		}
		cout<<"min_index : "<<min_index<<endl;
		*output_cloud = cluster_list[min_index];
	}
}

void check_cluster_normal_vector(CloudType::Ptr single_cluster, 
								 CloudType::Ptr output_cloud)
{
	for(size_t i = 0; i < single_cluster->points.size(); i++){
		// cout<<"single_cluster->points["<<i<<"].normal_x : "<<single_cluster->points[i].normal_x<<endl;
		// cout<<"single_cluster->points["<<i<<"].normal_y : "<<single_cluster->points[i].normal_y<<endl;
		// cout<<"single_cluster->points["<<i<<"].normal_z : "<<single_cluster->points[i].normal_z<<endl<<endl;
		if(single_cluster->points[i].normal_x < 1.0 && single_cluster->points[i].normal_y < 1.0){
			output_cloud->points.push_back(single_cluster->points[i]);
		}
	}
	// *output_cloud = *single_cluster;
}

void cluster(CloudType::Ptr input_cloud, 
			 CloudType::Ptr multi_cluster,
			 CloudType::Ptr single_cluster,
			 CloudType::Ptr output_cloud,
			 std::vector<pcl::PointIndices> &cluster_indices)
{
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	tree->setInputCloud(input_cloud);

	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(0.300);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input_cloud);
	ec.extract(cluster_indices);

	std::vector<CloudType> cluster_list;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
		CloudType::Ptr cluster_cloud (new CloudType);
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cluster_cloud->points.push_back(input_cloud->points[*pit]);
		}
		cout<<"cluster_cloud->points.size() : "<<cluster_cloud->points.size()<<endl;
		cluster_list.push_back(*cluster_cloud);
	}
	cout<<"cluster_list.size() : "<<cluster_list.size()<<endl;
		
	coloring_cluster(cluster_list);

	for(size_t i = 0;i<cluster_list.size();i++){
		*multi_cluster += cluster_list[i];
	}
	
	check_cluster_min_distance(cluster_list, single_cluster);

	check_cluster_normal_vector(single_cluster, output_cloud);
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
	// cout<<"PointCloud representing the planar component : "
		// <<output_cloud->width * output_cloud->height<<" data points."<<endl;
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

	// cout<<"Model coefficients : "<<coefficients->values[0]<<"\t"
								 // <<coefficients->values[1]<<"\t"
								 // <<coefficients->values[2]<<"\t"
								 // <<coefficients->values[3]<<endl;
	// cout<<"Model inliers : "<<inliers->indices.size()<<endl;
	size_t inliers_size = inliers->indices.size();
	for(size_t i = 0;i < inliers_size; i++){
		pointcloud->points[inliers->indices[i]].r = 255;
		pointcloud->points[inliers->indices[i]].g = 0;
		pointcloud->points[inliers->indices[i]].b = 0;
	}
}

void humanPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	cout<<"====================================================="<<endl;
	CloudType::Ptr pointcloud (new CloudType);
	pcl::fromROSMsg(*msg, *pointcloud);
	// cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	CloudType::Ptr removed_points (new CloudType);

	plane_segmentation(pointcloud, coefficients, inliers);
	plane_removal(pointcloud, removed_points, inliers);

	std::vector<pcl::PointIndices> cluster_indices;
	CloudType::Ptr multi_cluster (new CloudType);
	CloudType::Ptr single_cluster (new CloudType);
	CloudType::Ptr cluster_cloud (new CloudType);
	// cluster(removed_points, multi_cluster, single_cluster, cluster_cloud, cluster_indices);
	cluster(pointcloud, multi_cluster, single_cluster, cluster_cloud, cluster_indices);


	// calculate human cluster centroid
	PointType centroid;
	calculate_centroid(*cluster_cloud, &centroid);
	// cout<<"centroid : "<<centroid<<endl;
	CloudType::Ptr centroid_cloud (new CloudType);
	centroid_cloud->points.push_back(centroid);
	
	sensor_msgs::PointCloud2 centroid_cloud_pc2;
	pcl::toROSMsg(*centroid_cloud, centroid_cloud_pc2);
	centroid_cloud_pc2.header = msg->header;
	pub_centroid_cloud.publish(centroid_cloud_pc2);


	//plane segmentation (plane is segmented red color)
	sensor_msgs::PointCloud2 debug_pc;
	pcl::toROSMsg(*pointcloud, debug_pc);
	debug_pc.header = msg->header;
	pub_debug.publish(debug_pc);

	//plane extracted
	sensor_msgs::PointCloud2 debug_pc2;
	pcl::toROSMsg(*removed_points, debug_pc2);
	debug_pc2.header = msg->header;
	pub_debug2.publish(debug_pc2);

	//multiple clusters are colored in red, green, bule, black and white
	sensor_msgs::PointCloud2 debug_pc3;
	pcl::toROSMsg(*multi_cluster, debug_pc3);
	debug_pc3.header = msg->header;
	pub_debug3.publish(debug_pc3);

	//single cluster(extracted by distance from LiDAR)
	sensor_msgs::PointCloud2 debug_pc4;
	pcl::toROSMsg(*single_cluster, debug_pc4);
	debug_pc4.header = msg->header;
	pub_debug4.publish(debug_pc4);

	//single cluster(extracted by normal vector) <--- for now, human clustering is completed
	sensor_msgs::PointCloud2 human_points_pc2;
	pcl::toROSMsg(*cluster_cloud, human_points_pc2);
	human_points_pc2.header = msg->header;
	pub_human_points.publish(human_points_pc2);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_cluster");
	ros::NodeHandle n;

	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug", 1);
	pub_debug2 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug2", 1);
	pub_debug3 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug3", 1);
	pub_debug4 = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug4", 1);
	pub_human_points = n.advertise<sensor_msgs::PointCloud2>("/human_points", 1);
	pub_centroid_cloud = n.advertise<sensor_msgs::PointCloud2>("/human_points/centroid", 1);

	ros::Subscriber sub_humanpoints = n.subscribe("/human_points/candidate", 1, humanPointsCallback);

	cout<<"Here we go!!!"<<endl;

	ros::spin();

	return 0;
}
