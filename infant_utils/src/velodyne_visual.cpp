#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;

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

void rmGroundCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub4.publish(*msg);
}

void curvCallback(const sensor_msgs::PointCloud::Ptr& msg)
{
    msg->header.frame_id="velodyne2";
    msg->header.stamp   =ros::Time::now(); 
    pub5.publish(*msg);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "velodyne2");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/perfect_velodyne/normal", 10, pVelPointCallback);
    ros::Subscriber sub2 = n.subscribe("/velodyne_points", 10, velPointCallback);
    ros::Subscriber sub3 = n.subscribe("/velodyne_points/constraint", 10, velPointConsCallback);
    ros::Subscriber sub4 = n.subscribe("/rm_ground2", 10, rmGroundCallback);
    ros::Subscriber sub5 = n.subscribe("/curvature", 10, curvCallback);

    pub1 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2", 10);
    pub2 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2/normal", 10);
    pub3 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2/constraint", 10);
    pub4 = n.advertise<sensor_msgs::PointCloud2>("/velodyne2/rm_ground2", 10);
    pub5 = n.advertise<sensor_msgs::PointCloud>("/velodyne2/curvature", 10);

    ros::spin();
    return 0;
}
