/*
 * manual_calibration.cpp
 *
 *  Created on: 27.2.2014
 *      Author: ivelas
 */

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
// #include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <my_but_calibration_camera_velodyne/Image.h>
#include <my_but_calibration_camera_velodyne/Velodyne.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;
string VELODYNE_COLOR_TOPIC;

ros::Publisher pub;
ros::Publisher pub_debug;
ros::Publisher pub_debug2;
ros::Publisher pub_debug3;
ros::Publisher pub_full;
cv::Mat projection_matrix;

cv::Mat frame_rgb;
vector<float> DoF;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
	float p[12];
	float *pp = p;
	for (boost::array<double, 12ul>::const_iterator i = msg->P.begin(); i != msg->P.end(); i++){
		*pp = (float)(*i);
		pp++;
	}
	cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	frame_rgb = cv_ptr->image;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	// if no rgb frame for coloring:
	if (frame_rgb.data == NULL){
		cout<<"No rgb data!!!!"<<endl;
		return;
	}

	PointCloud<Velodyne::Point> pc;
	fromROSMsg(*msg, pc);

	// x := x, y := -z, z := y,
	Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI/2, -M_PI/2, 0);

	PointCloud<PointXYZ>* pcl_pc;
	pcl_pc = pointcloud.toPointsXYZ();
	sensor_msgs::PointCloud2 pc2;
	toROSMsg(*pcl_pc, pc2);
	pc2.header = msg->header;
	pub_debug.publish(pc2);

	Image::Image img(frame_rgb);
	Velodyne::Velodyne transformed = pointcloud.transform(DoF);
	PointCloud<Velodyne::Point> visible_points;
	// transformed.project(projection_matrix, Rect(0, 0, frame_rgb.cols, frame_rgb.rows), &visible_points);
	vector<int> index;
	transformed.project(projection_matrix, Rect(0, 0, frame_rgb.cols, frame_rgb.rows), &visible_points, &index);
	// for(size_t i=0;i<index.size();i++){
		// cout<<"index["<<i<<"] : "<<index[i]<<endl;
	// }

	sensor_msgs::PointCloud2 pc2_debug2;
	toROSMsg(visible_points, pc2_debug2);
	pc2_debug2.header = msg->header;
	pub_debug2.publish(pc2_debug2);
	// cout<<"visible_points : "<<visible_points.points.size()<<endl;

	cout<<"frame_rgb.cols : "<<frame_rgb.cols<<"typeid(frame_rgb.cols).name() : "<<typeid(frame_rgb.cols).name()<<endl;
	cout<<"frame_rgb.rows : "<<frame_rgb.rows<<"typeid(frame_rgb.rows).name() : "<<typeid(frame_rgb.rows).name()<<endl;

	Velodyne::Velodyne visible_scan(visible_points);

	PointCloud<PointXYZRGB> color_cloud = visible_scan.colour(frame_rgb, projection_matrix);

	// sensor_msgs::PointCloud2 pc2_color;
	// toROSMsg(color_cloud, pc2_color);
	// pc2_color.header = msg->header;
	// pub_debug3.publish(pc2_color);

	// reverse axix switching:
	Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI/2, 0, -M_PI/2);
	transformPointCloud(color_cloud, color_cloud, transf);
	

	PointCloud<PointXYZRGB> full_cloud;
	fromROSMsg(*msg, full_cloud);

	size_t full_cloud_size = full_cloud.points.size();
	for(size_t i=0;i<full_cloud_size;i++){
		full_cloud.points[i].r = 0;
		full_cloud.points[i].g = 0;
		full_cloud.points[i].b = 0;
	}
	
	size_t color_cloud_size = color_cloud.points.size();
	for(size_t i=0;i<color_cloud_size;i++){
		// cout<<"index["<<i<<"] : "<<index[i]<<endl;
		// cout<<"color_cloud.points["<<i<<"].x : "<<color_cloud.points[i].x<<endl;
		// cout<<"color_cloud.points["<<i<<"].y : "<<color_cloud.points[i].y<<endl;
		// cout<<"color_cloud.points["<<i<<"].z : "<<color_cloud.points[i].z<<endl;
		// cout<<"color_cloud.points["<<i<<"].r : "<<unsigned(color_cloud.points[i].r)<<endl;
		// cout<<"color_cloud.points["<<i<<"].g : "<<unsigned(color_cloud.points[i].g)<<endl;
		// cout<<"color_cloud.points["<<i<<"].b : "<<unsigned(color_cloud.points[i].b)<<endl;

		// cout<<"full_cloud.points["<<index[i]<<"].x : "<<full_cloud.points[index[i]].x<<endl;
		// cout<<"full_cloud.points["<<index[i]<<"].y : "<<full_cloud.points[index[i]].y<<endl;
		// cout<<"full_cloud.points["<<index[i]<<"].z : "<<full_cloud.points[index[i]].z<<endl;

		full_cloud.points[index[i]].r = color_cloud.points[i].r;
		full_cloud.points[index[i]].g = color_cloud.points[i].g;
		full_cloud.points[index[i]].b = color_cloud.points[i].b;

		// cout<<"full_cloud.points["<<index[i]<<"].r : "<<unsigned(full_cloud.points[index[i]].r)<<endl;
		// cout<<"full_cloud.points["<<index[i]<<"].g : "<<unsigned(full_cloud.points[index[i]].g)<<endl;
		// cout<<"full_cloud.points["<<index[i]<<"].b : "<<unsigned(full_cloud.points[index[i]].b)<<endl;
	}

	sensor_msgs::PointCloud2 color_cloud2;
	toROSMsg(color_cloud, color_cloud2);
	color_cloud2.header = msg->header;
	pub.publish(color_cloud2);

	sensor_msgs::PointCloud2 full_cloud2;
	toROSMsg(full_cloud, full_cloud2);
	full_cloud2.header = msg->header;
	pub_full.publish(full_cloud2);


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coloring_node");

	ros::NodeHandle n;
	n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic", VELODYNE_COLOR_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/6DoF", DoF);

	pub = n.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);
	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/velodyne_debug", 1);
	pub_debug2 = n.advertise<sensor_msgs::PointCloud2>("/velodyne_debug2", 1);
	pub_debug3 = n.advertise<sensor_msgs::PointCloud2>("/velodyne_debug3", 1);

	pub_full = n.advertise<sensor_msgs::PointCloud2>("/velodyne_colored_points/full", 1);

	// Subscribe input camera image
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe(CAMERA_FRAME_TOPIC, 10, imageCallback);

	ros::Subscriber info_sub = n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);

	ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>(VELODYNE_TOPIC, 1, pointCloudCallback);

	ros::spin();

	return EXIT_SUCCESS;
}
