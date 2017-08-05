#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/opencv.hpp"
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;

float cx;
float cy;
float f;

ros::Publisher pub;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
	cout<<"msg : "<<*msg<<endl;
	cx = msg->K[2];
	cy = msg->K[5];
	f = msg->K[0];
}

void depthToPoints(cv::Mat img, pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
	cout<<"aaa"<<endl;
	int width = img.cols;
	int height = img.rows;
	int index = 0;
	for(int y=0; y<height;y++){
		for(int x=0; x<width;x++){
			// cout<<"img.at<float>("<<y<<", "<<x<<") : "<<img.at<float>(y, x)<<endl;
			float dist = img.at<float>(y, x);
			if(dist>0){
				float Z = dist;
				float X = ((float)x - cx) * (dist / 1000.0) / (f / 1000.0);
				float Y = ((float)y - cy) * (dist / 1000.0) / (f / 1000.0);
				if(X <= 6 && Y <= 6 && Z <= 6){
					pc->points.resize(index+1);
					pc->points[index].z = Z;
					pc->points[index].x = X;
					pc->points[index].y = Y;
					index++;
				}

				// pc->points.resize(index+1);
				// pc->points[index].z = dist;
				// pc->points[index].x = ((float)x - cx) * (dist / 1000.0) / (f / 1000.0);
				// pc->points[index].y = ((float)y - cy) * (dist / 1000.0) / (f / 1000.0);
				// index++;
			}
		}
	}
	cout<<"finish!!"<<endl;
}

void showDepthImage(cv::Mat img){
	cv::Mat depth_image_output;
	cv::normalize(img, depth_image_output, 0, 1, cv::NORM_MINMAX);
	// cout<<"CV::NORM_MINMAX : "<<cv::NORM_MINMAX<<endl;
	cv::imshow("Depth Image", depth_image_output);
}


void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, float x, float y, float z, float roll, float pitch, float yaw){
	Eigen::Affine3f transform_matrix = pcl::getTransformation(x, y, z, roll, pitch, yaw);

	pcl::transformPointCloud(*input, *output, transform_matrix);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg){
	cout<<"Subscribe!!!"<<endl;

	try{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		cv::Mat depth_image = cv_ptr->image;

		// cv::resize(depth_image, depth_image, cv::Size(), 4, 4);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
		depthToPoints(depth_image, pointcloud);
		cout<<"pointcloud->points.size() : "<<pointcloud->points.size()<<endl;
		
		transform(pointcloud, pointcloud, 0, 0, 0, -M_PI/2, 0, -M_PI/2);

		sensor_msgs::PointCloud2 pc2;
		pcl::toROSMsg(*pointcloud, pc2);
		// pc2.header.frame_id = "/zed_current_frame";
		pc2.header.frame_id = "/zed_optical_frame";
		pc2.header.stamp = ros::Time::now();

		pub.publish(pc2);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'Target format'.", msg->encoding.c_str());
	}	

	// showDepthImage(depth_image);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "zed_depth2pointcloud");
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);
	// image_transport::Subscriber sub = it.subscribe("/camera/depth/resized_image", 1, depthCallback);
	image_transport::Subscriber subi_image = it.subscribe("/camera/depth/image_rect_color", 1, depthCallback);
	
	ros::Subscriber sub_info = n.subscribe("/zed/rgb/camera_info", 1, cameraInfoCallback);

	pub = n.advertise<sensor_msgs::PointCloud2>("/zed/points", 1);
	
	cout<<"Here we go!!"<<endl;

	// cv::namedWindow("Depth Image");
	// cv::startWindowThread();
	
	ros::spin();

	return 0;
}
