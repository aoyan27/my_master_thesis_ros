#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
// #include <camera_info_manager/camera_info_manager.h>
// #include <tf/tf.h>

#include "opencv2/opencv.hpp"

// #include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;
using namespace pcl;

string CAMERA_INFO_TOPIC;
string VELODYNE_COLOR_TOPIC;
vector<float> DoF;

cv::Mat projection_matrix;

ros::Publisher pub_debug;

std_msgs::Header img_header;

vector<int> bbox_array;

bool bbox_flag = false;

cv::Point cv_project(PointXYZRGB input, cv::Mat matrix){
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = input.x;
    pt_3D.at<float>(1) = input.y;
    pt_3D.at<float>(2) = input.z;
    pt_3D.at<float>(3) = 1.0f; // is homogenious coords. the point's 4. coord is 1

    cv::Mat pt_2D = matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;

    return cv::Point2f(x, y);
}

void project(cv::Mat matrix, Rect frame, PointCloud<PointXYZRGB>::Ptr input, PointCloud<PointXYZRGB>::Ptr output, vector<int>* index){
	size_t input_size = input->points.size();
	for (size_t i = 0; i < input_size; i++){
		
		// behind the camera
		if (input->points[i].z < 0.0){
			continue;
		}
		// cout<<"input->points["<<i<<"] : "<<input->points[i]<<endl;

		cv::Point xy = cv_project(input->points[i], projection_matrix);

		if (xy.inside(frame)){
			if (output != NULL){
				output->points.push_back(input->points[i]);
				index->push_back(i);
			}
		}
	}
}

void transform(PointCloud<PointXYZRGB>::Ptr input, PointCloud<PointXYZRGB>::Ptr output, float x, float y, float z, float roll, float pitch, float yaw)
{
	Eigen::Affine3f transform_matrix = getTransformation(x, y, z, roll, pitch, yaw);

	transformPointCloud(*input, *output, transform_matrix);
}

void transform_DoF(PointCloud<PointXYZRGB>::Ptr input, PointCloud<PointXYZRGB>::Ptr output, vector<float> dof)
{
	transform(input, output, dof[0], dof[1], dof[2], dof[3], dof[4], dof[5]);
}

void imageHeaderCallback(const std_msgs::HeaderConstPtr& msg){
	// cout<<"msg : "<<msg<<endl;
	img_header = *msg;
	cout<<"img_header : "<<endl<<img_header<<endl;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
	// cout<<"msg : "<<*msg<<endl;	
	float p[12];
	size_t P_size = 3 * 4;
	for(size_t i=0; i<P_size; i++){
		p[i] = msg->P[i];
		// cout<<"p["<<i<<"] : "<<p[i]<<endl;
	}
	cv::Mat(3, 4, CV_32FC1, p).copyTo(projection_matrix);
	// cout<<"projection_matrix : "<<projection_matrix<<endl;
}

void bboxCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	bbox_flag =true;

	size_t msg_size = msg->data.size();
	// cout<<"msg_size : "<<msg_size<<endl;
	bbox_array.clear();
	for(size_t i=0; i<msg_size;i++){
		bbox_array.push_back(2 * msg->data[i]);
		// cout<<"bbox_array["<<i<<"] : "<<bbox_array[i]<<endl;
	}
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	PointCloud<PointXYZRGB>::Ptr pcl_pc (new PointCloud<PointXYZRGB>);
	fromROSMsg(*msg, *pcl_pc);
	
	// cout<<"pcl_pc->points.size() : "<<pcl_pc->points.size()<<endl;

	if(bbox_flag){
		bbox_flag = false;
		PointCloud<PointXYZRGB>::Ptr pointcloud (new PointCloud<PointXYZRGB>);
		transform(pcl_pc, pointcloud, 0, 0, 0, M_PI/2, -M_PI/2, 0);
		PointCloud<PointXYZRGB>::Ptr transformed (new PointCloud<PointXYZRGB>);
		transform_DoF(pointcloud, transformed, DoF);
		
		PointCloud<PointXYZRGB>::Ptr visible_points (new PointCloud<PointXYZRGB>);
		vector<int> index;
		// project(projection_matrix, Rect(0, 0, 1920, 1080), transformed, visible_points, &index);
		cout<<"bbox_array[0] : "<<bbox_array[0]<<"\tbbox_array[1] : "<<bbox_array[1]<<"\tbbox_array[2] : "<<bbox_array[2]<<"\tbbox_array[3] : "<<bbox_array[3]<<endl;
		project(projection_matrix, Rect(bbox_array[0], bbox_array[1], bbox_array[2] - bbox_array[0], bbox_array[3] - bbox_array[1]), transformed, visible_points, &index);
		
		// cout<<"visible_points->points.size() : "<<visible_points->points.size()<<endl;

		PointCloud<PointXYZRGB>::Ptr human_points (new PointCloud<PointXYZRGB>);
		transform(visible_points, human_points, 0, 0, 0, -M_PI/2, 0, -M_PI/2);
		cout<<"human_points->points.size() : "<<human_points->points.size()<<endl;


		sensor_msgs::PointCloud2 debug_pc2;
		// toROSMsg(*pointcloud, debug_pc2);
		// toROSMsg(*transformed, debug_pc2);
		// toROSMsg(*visible_points, debug_pc2);
		toROSMsg(*human_points, debug_pc2);
		debug_pc2.header = msg->header;
		pub_debug.publish(debug_pc2);


	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_points_extractor");
	ros::NodeHandle n;
	n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic", VELODYNE_COLOR_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/6DoF", DoF);

	pub_debug = n.advertise<sensor_msgs::PointCloud2>("/human_points/debug", 1);

	ros::Subscriber sub_camerainfo = n.subscribe(CAMERA_INFO_TOPIC, 1, cameraInfoCallback);
	ros::Subscriber sub_points = n.subscribe(VELODYNE_COLOR_TOPIC, 1, pointCloudCallback);

	ros::Subscriber sub_header = n.subscribe("/image_header", 1, imageHeaderCallback);
	ros::Subscriber sub_bbox = n.subscribe("/bbox_array", 1, bboxCallback);

	ros::spin();


	return 0;
}
