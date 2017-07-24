#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static const string OPENCV_WINDOW = "Image Window";

class ImageConverter
{
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:
	ImageConverter()
		: it_(n_)
	{
		image_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect", 1, &ImageConverter::imageCallback, this);
		image_pub_ = it_.advertise("/output_image", 1);
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv bridge exception : %s", e.what());
			return;
		}

		cv::Mat hsv_image, color_mask, gray_image, cv_image2, cv_image3;
		cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
		cv::inRange(hsv_image, cv::Scalar(150, 100, 50, 0), cv::Scalar(180, 255, 255, 0), color_mask);

		cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);

		cv::cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);

		cv::Canny(gray_image, cv_ptr3->image, 15.0, 30.0, 3);

		cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0, 255, 0));

		cv::Mat cv_half_image, cv_half_image2, cv_half_image3;
		
		cv::resize(cv_ptr->image, cv_half_image, cv::Size(), 0.5, 0.5);
		cv::resize(gray_image, cv_half_image2, cv::Size(), 0.5, 0.5);
		cv::resize(cv_ptr3->image, cv_half_image3, cv::Size(), 0.5, 0.5);



		cv::imshow("Original Image", cv_half_image);
		cv::imshow("Result Image", cv_half_image2);
		cv::imshow("Edge Image", cv_half_image3);

		image_pub_.publish(cv_ptr3->toImageMsg());
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "image_converter");
	cv::namedWindow("Original Image");
	cv::namedWindow("Result Image");
	cv::namedWindow("Edge Image");
	cv::startWindowThread();
	ImageConverter ic;
	
	ros::spin();
	return 0;
}
