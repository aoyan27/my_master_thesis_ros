#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cout<<"Subscribe!!!!!"<<endl;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "my_image_transport_test");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
	ros::Rate loop_rate(10);

	// while(ros::ok()){
		
		// // pub.publish(cv_ptr->toImageMsg());

		// ros::spinOnce();
		// loop_rate.sleep();
	// }
	ros::spin();
	return 0;	
}
