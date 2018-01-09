#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/thread.hpp>

using namespace std;


ros::Publisher pub;

typedef pcl::PointNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;

#define RANGE_CONSTRAINT 7.5

void velPointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	CloudType::Ptr input_points (new CloudType);
	CloudType::Ptr output_points (new CloudType);
	pcl::fromROSMsg(*msg, *input_points);

	size_t input_size = input_points->points.size();
	for(size_t i = 0; i < input_size; i++){
		PointType temp_point;
		temp_point.x = input_points->points[i].x; 
		temp_point.y = input_points->points[i].y;
		temp_point.z = input_points->points[i].z;
		if((-1.0*RANGE_CONSTRAINT <= temp_point.x && temp_point.x <= RANGE_CONSTRAINT) \
				&& (-1.0*RANGE_CONSTRAINT <= temp_point.y && temp_point.y <= RANGE_CONSTRAINT))
		{
			output_points->points.push_back(temp_point);
		}
	}
	
	sensor_msgs::PointCloud2 output_pc2;
	pcl::toROSMsg(*output_points, output_pc2);
	// pcl::toROSMsg(*input_points, output_pc2);
	output_pc2.header = msg->header;
    pub.publish(output_pc2);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "velodyne_range_constraint");
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal", 10, velPointCallback);
    ros::Subscriber sub = n.subscribe("/velodyne_points", 10, velPointCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/constraint", 10);

	cout<<"Here we go!!"<<endl;

    ros::spin();
    return 0;
}
