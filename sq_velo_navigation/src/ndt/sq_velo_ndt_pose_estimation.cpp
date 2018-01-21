#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>

#include <iostream>
#include <math.h>
#include <time.h>
#include <stdio.h>

#include <boost/thread.hpp>

// #include "ray_casting.h"

using namespace std;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.3;
const float local_W = 20.0;
const float local_H = 20.0;

const float position_x = (min_x - local_W) / 2.0;
const float position_y = (min_y - local_H) / 2.0;


pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_data (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points__ (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 vis_output_points;


sensor_msgs::PointCloud2 vis_local_map_points;
sensor_msgs::PointCloud2 vis_velodyne_points;

nav_msgs::Odometry odom;

boost::mutex mutex_imu;
boost::mutex mutex_odom;
boost::mutex mutex_velodyne;


bool odom_sub_flag = false;
// bool odom_sub_flag = true;
bool velodyne_sub_flag = false;

float dt = 1.0;
float velocity = 0.0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

float dyaw = 0.0;

const float dyaw_threshold = 1.0;
// const float dyaw_threshold = 0.10;
// const float dyaw_threshold = 0.04;
// const float dyaw_threshold = 0.15;

const float init_x = 0.0;
const float init_y = 0.0;
// const float init_yaw = (-90.0) / 180.0 * M_PI;
// const float init_yaw = (-7.2) / 180.0 * M_PI;
const float init_yaw = (0.0) / 180.0 * M_PI;

void PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 *input, pcl::PointCloud<pcl::PointXYZ>::Ptr *output){
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl_conversions::toPCL(*input, pcl_cloud2);
	pcl::fromPCLPointCloud2(pcl_cloud2, **output);
}

void PointXYZ2PointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr *input, sensor_msgs::PointCloud2 *output){
	pcl::PCLPointCloud2 pcl_pointcloud2;
	pcl::toPCLPointCloud2(**input, pcl_pointcloud2);
	pcl_conversions::fromPCL(pcl_pointcloud2, *output);
}

float rotation_x(float x, float y, float theta){
	float x_r;
	// if(theta >= 0){
	x_r = cos(theta) * x - sin(theta) * y;
	// }
	// else{
		// x_r = cos(theta) * x - sin(theta) * y;
	// }
	return x_r;
}

float rotation_y(float x, float y, float theta){
	float y_r;
	// if(theta >= 0){
	y_r = sin(theta) * x + cos(theta) * y;
	// }
	// else{
		// y_r = -1 * sin(theta) * x + cos(theta) * y;
	// }
	return y_r;
}

void imu_callback(sensor_msgs::Imu::ConstPtr msg){
	boost::mutex::scoped_lock(mutex_imu);
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll_, pitch_, yaw_;
	m.getRPY(roll_, pitch_, yaw_);
	pitch = pitch_;
	dyaw = msg->angular_velocity.z;
}


void lcl_callback(nav_msgs::Odometry msg){
	boost::mutex::scoped_lock(mutex_odom);
	odom = msg;
	// cout<<"odom.pose.position.x : "<<odom.pose.pose.position.x<<endl;
	// cout<<"odom.pose.position.y : "<<odom.pose.pose.position.y<<endl;
	// cout<<"odom.pose.position.z : "<<odom.pose.pose.position.z<<endl;
	// cout<<"odom.pose.orientation.z : "<< odom.pose.pose.orientation.z<<endl;
	yaw = tf::getYaw(odom.pose.pose.orientation);
	odom_sub_flag = true;
}

void velodyne_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_velodyne);
	sensor_msgs::PointCloud2 temp = msg;
	temp.header.stamp = ros::Time::now();
	PointCloud2_to_PointXYZ(&temp, &velodyne_points__);
	
	velodyne_points_->points.clear();
	size_t velodyne_size = velodyne_points__->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = velodyne_points__->points[i].x; 
		temp_point.y = velodyne_points__->points[i].y;
		temp_point.z = velodyne_points__->points[i].z;
		if((-10.0 <= temp_point.x && temp_point.x <= 10.0) && (-10.0 <= temp_point.y && temp_point.y <= 10.0)){
			velodyne_points_->points.push_back(temp_point);
		}
	}

	// cout<<"velodyne_points_->points.size()"<<velodyne_points_->points.size()<<endl;
	velodyne_sub_flag = true;
}

void calc_z_offset(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cloud){
	vector<float> data_map(local_W / R * local_H / R, 100.0);
	vector<float> data_velodyne(local_W / R * local_H / R, 100.0);
	vector<float> data_z_offset(local_W / R * local_H / R, 0.0);
	size_t map_cloud_size = map_cloud->points.size();
	size_t velodyne_cloud_size = velodyne_cloud->points.size();
	size_t data_size = data_map.size();	

	for(size_t i = 0; i < map_cloud_size; i++){
		int x = int((map_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((map_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		
		if(map_cloud->points[i].z < data_map[num]){
			data_map[num] = map_cloud->points[i].z;
		}
		// cout<<"data_map["<<num<<"] : "<<data_map[num]<<endl;
	}

	for(size_t i = 0; i < velodyne_cloud_size; i++){
		int x = int((velodyne_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((velodyne_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		
		if(velodyne_cloud->points[i].z < data_velodyne[num]){
			data_velodyne[num] = velodyne_cloud->points[i].z;
		}
		// cout<<"data_velodyne["<<num<<"] : "<<data_velodyne[num]<<endl;
	}
	
	for(size_t i=0; i<data_map.size();i++){
		if(data_map[i] == 100.0){
			data_map[i] = 0.0;
		}
		if(data_velodyne[i] == 100.0){
			data_velodyne[i] = 0.0;
		}
	}
	
	for(size_t i=0; i < data_size; i++){
		data_z_offset[i] = data_map[i] - data_velodyne[i];
	}

	for(size_t i = 0; i<map_cloud_size; i++){
		int x = int((map_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((map_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		// cout<<"data_z_offset["<<num<<"] : "<<data_z_offset[num]<<endl;
		map_cloud->points[i].z -= data_z_offset[num];
	}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "ndt_pose_estimation");

	ros::NodeHandle n;

//	string filename = "/home/amsl/obs_cloud_data_robosym2016_filtered.pcd";
	string filename = "/home/amsl/obs_cloud_data_d_kan_velo90test_filtered.pcd";
	n.getParam("map3d/rm_gnd4ndt", filename);
	// string filename = "/home/amsl/map_tsukuba_all2.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba.pcd";
	// string filename = "/home/amsl/obs_map.pcd";
	// string filename = "/home/amsl/obs_cloud_data_ikuta.pcd";
	// string filename = "/home/amsl/obs_cloud_data_ikuta_filtered.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba_filtered.pcd";
	// string filename = "/home/amsl/obs_cloud_data_robosym2016_filtered.pcd";
	if(pcl::io::loadPCDFile (filename, *global_map_data) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}

	// ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl2", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl3", 1, lcl_callback);
	ros::Subscriber sub3 = n.subscribe("/lcl5", 1, lcl_callback);

	ros::Subscriber sub5 = n.subscribe("/imu/data", 1, imu_callback);

	// ros::Subscriber sub6 = n.subscribe("/velodyne_points", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/rm_ground2", 1, velodyne_callback);
	ros::Subscriber sub6 = n.subscribe("/velodyne_obstacles", 1, velodyne_callback);
	


	ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/vis_velodyne_poitns", 1);
	ros::Publisher vis_pub_3d_local_map = n.advertise<sensor_msgs::PointCloud2>("/vis_3d_local_map", 1);
	ros::Publisher vis_pub_output = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);

	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);
	ros::Publisher pub_vis_lcl_ndt = n.advertise<nav_msgs::Odometry>("/vis_lcl_ndt", 10);

	ros::Publisher pub_fitness_score = n.advertise<std_msgs::Float64>("/fitness_score", 10);

	clock_t start, end;
	
	int count = 0;
	int count_ = 0;

	float l = 0.0;
	float x = 0.0;
	float y = 0.0;

	float fai = 0.0;
	float theta = 0.0;
	
	float odom_x = init_x;
	float odom_y = init_y;
	float odom_yaw = init_yaw;


	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);

	ros::Rate loop_rate(10);
	
	nav_msgs::Odometry vis_odom;

	std_msgs::Float64 fitness_score;

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_target;
	voxel_filter_target.setLeafSize(0.3, 0.3, 0.3);
	voxel_filter_target.setInputCloud(global_map_data);
	voxel_filter_target.filter(*filtered_target_points);
	ndt.setInputTarget(filtered_target_points);
	
	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	// cout<<"t : "<<t<<endl;
	
	double l_roll, l_pitch, l_yaw;

	while(ros::ok()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points (new pcl::PointCloud<pcl::PointXYZ>);
		{
			boost::mutex::scoped_lock(mutex_velodyne);
			velodyne_points = velodyne_points_;
		}
		cout<<"NDT Start!!!!"<<endl;
		
		vis_velodyne_points.header.stamp = ros::Time::now();	
		vis_local_map_points.header.stamp = ros::Time::now();	
		
		if(velodyne_sub_flag && odom_sub_flag){
			cout<<"just do it !!!"<<endl;
			
			PointXYZ2PointCloud2(&velodyne_points, &vis_velodyne_points);
			PointXYZ2PointCloud2(&global_map_data, &vis_local_map_points);
			vis_velodyne_points.header.frame_id = "/velodyne";
			vis_local_map_points.header.frame_id = "/map";
			vis_pub_velodyne_points.publish(vis_velodyne_points);
			vis_pub_3d_local_map.publish(vis_local_map_points);
			

			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setLeafSize(0.3, 0.3, 0.3);
			voxel_filter.setInputCloud(velodyne_points);
			voxel_filter.filter(*filtered_points);
			
		
			ndt.setInputSource(filtered_points);

			Eigen::AngleAxisf init_rotation(yaw + init_yaw, Eigen::Vector3f::UnitZ());
			Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"init_guess : "<<init_guess<<endl;


			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			// cout<<"output_points initialize!!!!!"<<endl;
			cout<<endl;
			// cout<<"pitch : "<<pitch<<endl;	
			cout<<"yaw : "<<yaw<<endl;	
			cout<<"dyaw : "<<dyaw<<endl;	
			cout<<"count_ : "<<count_<<endl;
			count_ ++;
			cout<<"count : "<<count<<endl;
			count ++;
			if(fabs(dyaw) < dyaw_threshold){
				start = clock();
				cout<<"start : "<<start<<endl;
				ndt.omp_align(*output_points, init_guess);
				cout<<"Just Finish NDT !!!"<<endl;
				end = clock();
				cout<<"end : "<<end<<endl;
				PointXYZ2PointCloud2(&output_points, &vis_output_points);

				// cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
				cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
				fitness_score.data = ndt.getFitnessScore();
				printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
				dt =  (double)(end-start) / CLOCKS_PER_SEC;
				cout<<"ndt.getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
				t = ndt.getFinalTransformation();
				// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
				
				
				tf::Matrix3x3 mat_l;
				mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
							   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
							   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
				
				mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);

				cout<<"t(0,3) : "<<t(0, 3)<<endl;
				cout<<"t(1,3) : "<<t(1, 3)<<endl;
				cout<<"l_roll : "<<l_roll<<endl;
				cout<<"l_pitch : "<<l_pitch<<endl;
				cout<<"l_yaw : "<<l_yaw - init_yaw<<endl;

				odom.pose.pose.position.x = t(0, 3);
				odom.pose.pose.position.y = t(1, 3);

				odom.pose.pose.orientation.z = l_yaw - init_yaw;

				vis_odom = odom;
				
				vis_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom.pose.pose.orientation.z);
			}
			else{
				fitness_score.data = 500.0;
			}

			pub_vis_lcl_ndt.publish(vis_odom);
			pub_lcl_ndt.publish(odom);
			vis_output_points.header.frame_id = "/map";
			vis_pub_output.publish(vis_output_points);
			pub_fitness_score.publish(fitness_score);

			odom_sub_flag = false;
			velodyne_sub_flag = false;
		}

		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
