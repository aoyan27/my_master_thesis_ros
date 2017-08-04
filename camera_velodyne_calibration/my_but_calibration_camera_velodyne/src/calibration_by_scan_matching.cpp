#include <ros/ros.h>

#include <tf/tf.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <stdio.h>

using namespace std;

string OUTPUT_FILE;
string VELODYNE_FILE;
string ZED_FILE;
string OUTPUT_PCD;
vector<float> init_DoF;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibration_by_scan_matching");
	ros::NodeHandle n;
	
	cout<<"target : zed, source : velodyne"<<endl;

	n.getParam("/output_filename", OUTPUT_FILE);
	cout<<"output_filename : "<<OUTPUT_FILE<<endl;

	n.getParam("/input_velodyne_filename", VELODYNE_FILE);
	cout<<"input_velodyne_filename : "<<VELODYNE_FILE<<endl;

	n.getParam("/input_zed_filename", ZED_FILE);
	cout<<"input_zed_filename : "<<ZED_FILE<<endl;
	
	n.getParam("/output_pcd", OUTPUT_PCD);
	cout<<"output_pcd : "<<OUTPUT_PCD<<endl;

	n.getParam("/init_6DoF", init_DoF);
	for(size_t i=0; i<init_DoF.size();i++){
		cout<<"init_6DoF : "<<init_DoF[i]<<endl;
	}

	// Loading first pcd file.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (ZED_FILE, *target_cloud) == -1){
		PCL_ERROR("Couldn't read file.\n");
		return -1;
	}
	cout<<"Loaded "<<target_cloud->size()<<" data points from target pcd file."<<endl;

	// Loading second pcd file.
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (VELODYNE_FILE, *input_cloud) == -1){
		PCL_ERROR("Couldn't read file.\n");
		return -1;
	}
	cout<<"Loaded "<<input_cloud->size()<<" data points from source pcd file."<<endl;
	
	Eigen::AngleAxisf rot(0.0, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f trans(init_DoF[0], init_DoF[1], init_DoF[2]);
	Eigen::Matrix4f sample_matrix = (trans * rot).matrix();
	pcl::transformPointCloud(*input_cloud, *input_cloud, sample_matrix);
	pcl::io::savePCDFileASCII ("/home/amsl/ros_catkin_ws/src/master_thesis/camera_velodyne_calibration/my_but_calibration_camera_velodyne/pcd_data/sample_input.pcd", *input_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);

	cout<<"Filtered cloud contains "<<filtered_cloud->size()<<" data points from source pcd file."<<endl;


	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon(0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize(0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution(1.0);

	// Setting max number of registration iterations.
	ndt.setMaximumIterations(35);

	// Setting point cloud to be aligned.
	ndt.setInputSource(filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(target_cloud);

	Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(init_DoF[0], init_DoF[1], init_DoF[2]);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();


	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud, init_guess);

	cout<<"Normal Distributions Transform has converged:"<<ndt.hasConverged()<<", score: "<<ndt.getFitnessScore()<<endl;

	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	t = ndt.getFinalTransformation();
	cout<<t<<endl;

	tf::Matrix3x3 mat;
	mat.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
				   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
				   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw, 1);
	// cout<<"x : "<<t(0, 3)<<endl;
	// cout<<"y : "<<t(1, 3)<<endl;
	// cout<<"z : "<<t(2, 3)<<endl;
	// cout<<"roll : "<<roll<<endl;
	// cout<<"pitch : "<<pitch<<endl;
	// cout<<"yaw : "<<yaw<<endl;
	
	printf("6DoF: [%f, %f, %f, %f, %f, %f]\n", t(0, 3), t(1, 3), t(2, 3), roll, pitch, yaw);
	
	// Saving transformed input cloud.
	pcl::io::savePCDFileASCII (OUTPUT_PCD, *output_cloud);



	return 0;
}
