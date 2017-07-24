#include <ros/ros.h>

#include <pcl/pcl_config.h>

using namespace std;


int main(int argc, char** argv){
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	
	cout<<PCL_VERSION_PRETTY<<endl;

}
