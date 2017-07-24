#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace ros;
using namespace pcl;

PointCloud<PointSurfel>::Ptr colored_pcl_pc (new PointCloud<PointSurfel>);
PointCloud<PointSurfel>::Ptr normal_pcl_pc (new PointCloud<PointSurfel>);
PointCloud<PointSurfel>::Ptr output (new PointCloud<PointSurfel>);

bool colored_flag = false;
bool normal_flag = false;

void colored_callback(sensor_msgs::PointCloud2 msg){
	colored_flag = true;
	fromROSMsg(msg, *colored_pcl_pc);
	cout<<"colored_pcl_pc->points.size() : "<<colored_pcl_pc->points.size()<<endl;
}

void normal_callback(sensor_msgs::PointCloud2 msg){
	normal_flag = true;
	fromROSMsg(msg, *normal_pcl_pc);
	cout<<"normal_pcl_pc->points.size() : "<<normal_pcl_pc->points.size()<<endl;
}

void integrate_points(PointCloud<PointSurfel>::Ptr colored_pc, PointCloud<PointSurfel>::Ptr normal_pc){
	cout<<"!!!integrate!!!"<<endl;

	// *output = *colored_pc + *normal_pc;
	*output = *colored_pc;
	cout<<"output->points.size() : "<<output->points.size()<<endl;
}

int main(int argc, char** argv){
	init(argc, argv, "integrate_points_color_normal");
	NodeHandle n;
	
	Subscriber sub_colored = n.subscribe("/velodyne_colored_points/full", 1, colored_callback);
	Subscriber sub_normal = n.subscribe("/perfect_velodyne/normal", 1, normal_callback);

	Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_pointcloud/integrate", 1);
	
	Rate loop_rate(20);
	
	while(ros::ok()){
		if(colored_flag && normal_flag){
			cout<<"!!!!!!Now processing!!!!!!!"<<endl;
			// integrate_points(colored_pcl_pc, normal_pcl_pc);
			
			// sensor_msgs::PointCloud2 pc2;
			// toROSMsg(*output, pc2);

			// pub.publish(pc2);
			
			colored_flag = false;
			normal_flag = false;
		}
		
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
