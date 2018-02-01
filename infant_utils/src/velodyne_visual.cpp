#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

void pVelPointCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub2.publish(*msg);
}

void velPointCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub1.publish(*msg);
}

void velPointConsCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub3.publish(*msg);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "velodyne2");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/perfect_velodyne/normal", 10, pVelPointCallback);
    ros::Subscriber sub2 = n.subscribe("/velodyne_points", 10, velPointCallback);
    ros::Subscriber sub3 = n.subscribe("/velodyne_points/constraint", 10, velPointConsCallback);

    pub1 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2", 10);
    pub2 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2/normal", 10);
    pub3 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2/constraint", 10);

    ros::spin();
    return 0;
}
