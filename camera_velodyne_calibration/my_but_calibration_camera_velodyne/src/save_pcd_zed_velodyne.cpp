#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <stdio.h>

using namespace std;

string VELODYNE_TOPIC;
string ZED_TOPIC;
string VELODYNE_FILE;
string ZED_FILE;

pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_pc (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr zed_pc (new pcl::PointCloud<pcl::PointXYZ>);

bool velodyne_flag = false;
bool zed_flag = false;

void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	velodyne_flag = true;
	pcl::fromROSMsg(*msg, *velodyne_pc);
	// cout<<"velodyne_pc->points.size() : "<<velodyne_pc->points.size()<<endl;
}

void zedCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	zed_flag = true;
	pcl::fromROSMsg(*msg, *zed_pc);
	// cout<<"zed_pc->points.size() : "<<zed_pc->points.size()<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_pcd_zed_velodyne");
	ros::NodeHandle n;

	n.getParam("/velodyne_topic", VELODYNE_TOPIC);
	n.getParam("/zed_topic", ZED_TOPIC);
	n.getParam("/input_velodyne_filename", VELODYNE_FILE);
	n.getParam("/input_zed_filename", ZED_FILE);

	// cout<<"input_velodyne_filename : "<<VELODYNE_FILE<<endl;
	// cout<<"input_zed_filename : "<<ZED_FILE<<endl;

	ros::Subscriber sub_velodyne = n.subscribe(VELODYNE_TOPIC, 1, velodyneCallback);
	ros::Subscriber sub_zed = n.subscribe(ZED_TOPIC, 1, zedCallback);

	ros::Rate loop_rate(10);
	
	int count = 0;
	printf("Here We go!!");
	while(ros::ok()){
		if(velodyne_flag && zed_flag){
			if(count%2){
				printf("\rNow waiting for 'Ctr+C'. velodyne and zed subscribed!! □ ■ ");
			}
			else{
				printf("\rNow waiting for 'Ctr+C'. velodyne and zed subscribed!! ■ □ ");
			}
		}
		else{
			if(count%2){
				printf("\rNow waiting for 'Ctr+C'. □ ■ ");
			}
			else{
				printf("\rNow waiting for 'Ctr+C'. ■ □ ");
			}
			
		}
		fflush(stdout);
		
		count++;

		ros::spinOnce();
		loop_rate.sleep();
	}

	cout<<endl;
	if(velodyne_flag && zed_flag){
		pcl::io::savePCDFileASCII(VELODYNE_FILE, *velodyne_pc);
		cout<<"saved velodyne_pointcloud ---> "<<VELODYNE_FILE<<endl;
		pcl::io::savePCDFileASCII(ZED_FILE, *zed_pc);
		cout<<"saved zed_pointcloud ---> "<<ZED_FILE<<endl;
	}
	else{
		cout<<"Faild!!!"<<endl;
	}
	
	return 0;
}
