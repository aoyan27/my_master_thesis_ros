#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


ros::Publisher pub;
sensor_msgs::PointCloud2 vel2;

void velPointCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub.publish(*msg);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "velodyne2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal", 10, velPointCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne2", 10);

    ros::spin();
    return 0;
}
