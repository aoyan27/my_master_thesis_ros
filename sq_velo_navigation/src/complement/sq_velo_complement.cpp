//LOCALIZATIONS
//s.shimizu
//t.hagiwara
//
//azimuth estimation + odometryでのlocalization
//tf名は
//		header	/map 
//		child	/matching_base_link
//
														

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <catkin_ceres_msgs/AMU_data.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

#include <stdio.h>
#include <iostream>

#include <complement/localization.h>

using namespace std;	

//const size_t N = 3;
// const size_t N = 4;
const size_t N = 5;
bool is_started = true;
bool DGauss_flag = false;
bool DGauss_yaw_flag = false;
bool ekf_flag = false;
bool ndt_flag = false;
bool ekf_ndt_flag = false;
bool init_flag=false;
Localization lcl[N];
//double initialpose[3] = { 95.278519 , -18.005138 , ( 173.7 * M_PI/ 180.0) };//ikucha
geometry_msgs::Pose2D init_pose;
nav_msgs::Odometry gps_odom_msgs_;
nav_msgs::Odometry gps_msgs;
nav_msgs::Odometry DGauss_pose;
nav_msgs::Odometry DGauss_yaw;
nav_msgs::Odometry lcl_ini;
nav_msgs::Odometry ekf_odom;
nav_msgs::Odometry ndt_odom;
nav_msgs::Odometry ekf_ndt_odom;



template <class T>
void getParam(ros::NodeHandle &n, string param, T &val, bool* flag)
{
    // string str;
    double prm;
    if(!n.hasParam(param)){
        std::cout << param << " don't exist." << std::endl;
        *flag=true;
    }

    // if(!n.getParam(param, str)){
    if(!n.getParam(param, prm)){
        std::cout << "NG" << std::endl;
        *flag=true;
    }
    // std::stringstream ss(str);
    // T rsl;
    // ss >> rsl;
    // val = rsl;
    val = prm;
    std::cout << param << " = " << prm << std::endl;
}

void setInitPose(ros::NodeHandle &n, geometry_msgs::Pose2D& init_pose)
{
    bool error_flag=false;
    getParam(n, "init_x", init_pose.x, &error_flag);
    getParam(n, "init_y", init_pose.y, &error_flag);
    getParam(n, "init_yaw", init_pose.theta, &error_flag);
	init_flag=true;
}


void imu_callback(sensor_msgs::Imu::ConstPtr msg){
	is_started = true;
	fflush(stdout);
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll_, pitch_, yaw_;
	m.getRPY(roll_, pitch_, yaw_);
	lcl[0].pitch = pitch_;
	lcl[1].pitch = pitch_;
	lcl[2].pitch = pitch_;
	lcl[3].pitch = pitch_;
	lcl[4].pitch = pitch_;

	lcl[1].w = msg->angular_velocity.z;
	lcl[2].w = msg->angular_velocity.z;
	lcl[4].w = msg->angular_velocity.z;

	lcl[1].altering();
	lcl[2].altering();

	lcl[4].altering();
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	is_started = true;
	printf("\r□ ■ □");
	fflush(stdout);
	lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
	lcl[2].v = msg->twist.twist.linear.x;
	lcl[3].v = msg->twist.twist.linear.x;
	lcl[4].v = msg->twist.twist.linear.x;

	lcl[0].w = msg->twist.twist.angular.z;
	//lcl[1].w += 0.00222222;
	lcl[0].altering();
//	lcl[1].showState();

//	cout<<msg->pose.pose.position.x<<", "<<msg->pose.pose.position.y<<endl;
}

// void azm_callback(sensor_msgs::Imu::ConstPtr msg){	
	// if (init_flag){
		// is_started = true;
		// printf("\r□ □ ■");
		// fflush(stdout);
		// lcl[2].yaw = (init_pose.theta + msg->orientation.z)*M_PI / 180.0;
		// lcl[2].altering();
		// lcl[3].yaw = (init_pose.theta + msg->orientation.z)*M_PI / 180.0;
		// lcl[3].altering2();
// //		lcl[2].showState();
	// }
// }
/*
void gps_callback(const nav_msgs::OdometryConstPtr msg)
{
	gps_msgs = *msg;
	lcl[2].x = gps_msgs.pose.pose.position.x;
	lcl[2].y = gps_msgs.pose.pose.position.y;
}
*/

void DGauss_callback(const nav_msgs::Odometry msg){
	DGauss_pose.pose.pose.position.x = msg.pose.pose.position.x;
	DGauss_pose.pose.pose.position.y = msg.pose.pose.position.y;
	DGauss_flag = true;
}

void DGauss_yaw_callback(const nav_msgs::Odometry msg){
	DGauss_yaw.pose.pose.orientation.z = msg.pose.pose.orientation.z;
	DGauss_yaw_flag = true;
}

void ekfCallback(const nav_msgs::Odometry msg){
	// ekf_odom.pose.pose.position = msg.pose.pose.position;
	// ekf_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
	ekf_odom = msg;

	ekf_flag = true;
}

void ndt_callback(const nav_msgs::Odometry msg){
	ndt_odom = msg;
	ndt_flag = true;
}

void ekf_ndt_callback(const nav_msgs::Odometry msg){
	ekf_ndt_odom = msg;
	ekf_ndt_flag = true;
}

// void ekf_DgaussAndNDT_callback(const nav_msgs::Odometry msg){
	// ekf_ndt_odom = msg;
	// // lcl[1].x = ekf_ndt_odom.pose.pose.position.x;
	// // lcl[1].y = ekf_ndt_odom.pose.pose.position.y;
	// // lcl[1].yaw = ekf_ndt_odom.pose.pose.orientation.z;
    // //
	// // lcl[4].x = ekf_ndt_odom.pose.pose.position.x;
	// // lcl[4].y = ekf_ndt_odom.pose.pose.position.y;
	// // lcl[4].yaw = ekf_ndt_odom.pose.pose.orientation.z;
	// ekf_ndt_flag = true;
// }

int main (int argc, char** argv)
{
	ros::init(argc, argv, "localization_test");
  	ros::NodeHandle n;
	ros::Rate roop(100);
	
    setInitPose(n, init_pose);
	
	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		lcl[i].yaw = init_pose.theta*M_PI/180.0;
	}
	//previous version//
	//for(size_t i=0;i<N;i++){
	//	lcl[i].x =   initialpose[0];
	//	lcl[i].y =   initialpose[1];
	//	lcl[i].yaw = initialpose[2];
	//}
	
	for(size_t i=0;i<N;i++)
		lcl[i].start();
	
	lcl_ini.pose.pose.position.x = init_pose.x;
	lcl_ini.pose.pose.position.y = init_pose.y;
	lcl_ini.pose.pose.orientation.z = init_pose.theta*M_PI/180.0;//初期方位[rad]が入っている
	//lcl_ini.pose.pose.position.x = initialpose[0];
	//lcl_ini.pose.pose.position.y = initialpose[1];
	//lcl_ini.pose.pose.orientation.z = initialpose[2];//初期方位[rad]が入っている

	vector<ros::Publisher> pub;
	vector<sensor_msgs::PointCloud> pc;

	pub.resize(N);
	pc.resize(N);

	pub[0] = n.advertise<sensor_msgs::PointCloud>("/odm_lcl", 100);
	pub[1] = n.advertise<sensor_msgs::PointCloud>("/amu_lcl", 100);
	pub[2] = n.advertise<sensor_msgs::PointCloud>("/azm_lcl", 100);

	ros::Subscriber odm_sub = n.subscribe("/odom", 100, odom_callback);
	ros::Subscriber imu_sub = n.subscribe("/imu/data", 100, imu_callback);
	ros::Subscriber DGauss_sub = n.subscribe("/gauss_sphere/pose", 100, DGauss_callback);
	ros::Subscriber DGauss_yaw_sub = n.subscribe("/gauss_sphere/yaw", 100, DGauss_yaw_callback);
	ros::Subscriber ekf_sub = n.subscribe("/ekf_lcl", 100, ekfCallback);
	ros::Subscriber ndt_sub = n.subscribe("/lcl_ndt", 100, ndt_callback);
	ros::Subscriber ekf_ndt_sub = n.subscribe("/ekf_ndt", 100, ekf_ndt_callback);
	// ros::Subscriber ekf_ndt_sub = n.subscribe("/ekf_DgaussAndNDT", 100, ekf_DgaussAndNDT_callback);


//	ros::Subscriber gps_sub = n.subscribe("gps/xy", 100, gps_callback);
//	ros::Subscriber odm_sub = n.subscribe("/tinypower/state", 100, odom_callback);
	
	ros::Publisher pub_lcl = n.advertise<nav_msgs::Odometry>("/lcl", 10);
	ros::Publisher pub_lcl2 = n.advertise<nav_msgs::Odometry>("/lcl2", 10);
	ros::Publisher pub_lcl3 = n.advertise<nav_msgs::Odometry>("/lcl3", 10);
	ros::Publisher pub_lcl4 = n.advertise<nav_msgs::Odometry>("/lcl4", 10);
	ros::Publisher pub_lcl5 = n.advertise<nav_msgs::Odometry>("/lcl5", 10);
	ros::Publisher pub_lcl_ini = n.advertise<nav_msgs::Odometry>("/lcl/initial_pose", 10);
	
	ros::Publisher pub_vis_lcl = n.advertise<nav_msgs::Odometry>("/vis_lcl", 10);
	ros::Publisher pub_vis_lcl4 = n.advertise<nav_msgs::Odometry>("/vis_lcl4", 10);
	// ros::Publisher pub_vis_ekf_ndt = n.advertise<nav_msgs::Odometry>("/vis_ekf_ndt", 10);
	
	tf::TransformBroadcaster br;
	tf::Transform transform;
	nav_msgs::Odometry lcl_;
	lcl_.header.frame_id = "map";
	lcl_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry lcl2_;
	lcl2_.header.frame_id = "map";
	lcl2_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry lcl3_;
	lcl3_.header.frame_id = "map";
	lcl3_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry lcl4_;
	lcl4_.header.frame_id = "map";
	lcl4_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry lcl5_;
	lcl5_.header.frame_id = "map";
	lcl5_.child_frame_id = "matching_base_link";

	nav_msgs::Odometry vis_lcl_;
	vis_lcl_.header.frame_id = "map";
	vis_lcl_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry vis_lcl4_;
	vis_lcl4_.header.frame_id = "map";
	vis_lcl4_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry vis_ekf_ndt;
	vis_ekf_ndt.header.frame_id = "map";
	vis_ekf_ndt.child_frame_id = "matching_base_link";

	unsigned int cnt = 0;


//	FILE *fp_1;
	FILE *fp_2;
	fp_2 = fopen("./position.csv","w");

	fprintf(fp_2, "W.O._x,W.O_y,G.O._x,G.O._y,Pro._x,Pro._y\n");


	while(ros::ok()){
		if(is_started){
			if(cnt % 50 == 0){
				fprintf(fp_2, "%lf,%lf,%lf,%lf,%lf,%lf\n", lcl[0].x, lcl[0].y, lcl[1].x, lcl[1].y, lcl[2].x, lcl[2].y);
				cnt = 0;
			}
			if(DGauss_flag){
				lcl[1].x = DGauss_pose.pose.pose.position.x;
				lcl[1].y = DGauss_pose.pose.pose.position.y;
				// ekf_odom.pose.pose.position.x = DGauss_pose.pose.pose.position.x;
				// ekf_odom.pose.pose.position.y = DGauss_pose.pose.pose.position.y;
				//lcl[1].yaw += DGauss_pose.pose.pose.orientation.z;
				
				cout<<"DGauss_pose.pose.pose.position.x : "<<DGauss_pose.pose.pose.position.x<<endl;
				cout<<"DGauss_pose.pose.pose.position.y : "<<DGauss_pose.pose.pose.position.y<<endl;
				//cout<<"yaw : "<<lcl[1].yaw<<endl;
				
				DGauss_flag = false;
			}
			if(DGauss_yaw_flag){
				//lcl[1].x = DGauss_pose.pose.pose.position.x;
				//lcl[1].y = DGauss_pose.pose.pose.position.y;
				lcl[1].yaw = DGauss_yaw.pose.pose.orientation.z;
				lcl[4].yaw = DGauss_yaw.pose.pose.orientation.z;
				// ekf_ndt_odom.pose.pose.orientation.z += DGauss_yaw.pose.pose.orientation.z;
				//cout<<"DGauss_pose.pose.pose.position.x : "<<DGauss_pose.pose.pose.position.x<<endl;
				//cout<<"DGauss_pose.pose.pose.position.y : "<<DGauss_pose.pose.pose.position.y<<endl;
				cout<<"yaw : "<<lcl[1].yaw<<endl;
				
				DGauss_yaw_flag = false;
			}
			if(ndt_flag){
				lcl[4].x = ndt_odom.pose.pose.position.x;
				lcl[4].y = ndt_odom.pose.pose.position.y;
				// lcl[4].yaw = ndt_odom.pose.pose.orientation.z;
				ndt_flag = false;
			}
			// D-Gauss
			lcl_.header.stamp = ros::Time::now();
			lcl_.pose.pose.position.x = lcl[1].x;
			lcl_.pose.pose.position.y = lcl[1].y;
			lcl_.pose.pose.position.z = 0.0;
			lcl_.pose.pose.orientation.z = lcl[1].yaw;
			// lcl_.pose.pose.orientation.z = ekf_ndt_odom.pose.pose.orientation.z;
			
			vis_lcl_.header.stamp = ros::Time::now();
			vis_lcl_.pose.pose.position.x = lcl[1].x;
			vis_lcl_.pose.pose.position.y = lcl[1].y;
			vis_lcl_.pose.pose.position.z = 0.0;
			vis_lcl_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(lcl_.pose.pose.orientation.z);
			//lcl_.pose.pose.position.x = lcl[2].x;
			//lcl_.pose.pose.position.y = lcl[2].y;
			//lcl_.pose.pose.position.z = 0.0;
			//lcl_.pose.pose.orientation.z = lcl[2].yaw;
					
			pub_lcl.publish(lcl_);
			pub_vis_lcl.publish(vis_lcl_);
			pub_lcl_ini.publish(lcl_ini);
			
			// Gyro-Odometry
			lcl2_.header.stamp = ros::Time::now();
			lcl2_.pose.pose.position.x = lcl[2].x;
			lcl2_.pose.pose.position.y = lcl[2].y;
			lcl2_.pose.pose.position.z = 0.0;
			lcl2_.pose.pose.orientation.z = lcl[2].yaw;
			pub_lcl2.publish(lcl2_);
			
			// Wheel Odometry
			lcl3_.header.stamp = ros::Time::now();
			lcl3_.pose.pose.position.x = lcl[0].x;
			lcl3_.pose.pose.position.y = lcl[0].y;
			lcl3_.pose.pose.position.z = 0.0;
			lcl3_.pose.pose.orientation.z = lcl[0].yaw;
			pub_lcl3.publish(lcl3_);
			
			//NDT
			lcl4_.header.stamp = ros::Time::now();
			lcl4_.pose.pose.position.x = lcl[4].x;
			lcl4_.pose.pose.position.y = lcl[4].y;
			lcl4_.pose.pose.position.z = 0.0;
			lcl4_.pose.pose.orientation.z = lcl[4].yaw;
			// lcl4_.pose.pose.orientation.z = ekf_ndt_odom.pose.pose.orientation.z;
			pub_lcl4.publish(lcl4_);
			
			vis_lcl4_.header.stamp = ros::Time::now();
			vis_lcl4_.pose.pose.position.x = lcl[4].x;
			vis_lcl4_.pose.pose.position.y = lcl[4].y;
			vis_lcl4_.pose.pose.position.z = 0.0;
			vis_lcl4_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(lcl4_.pose.pose.orientation.z);
			pub_vis_lcl4.publish(vis_lcl4_);
			
			// EKF
			lcl5_.header.stamp = ros::Time::now();
			lcl5_.pose.pose.position.x = ekf_ndt_odom.pose.pose.position.x;
			lcl5_.pose.pose.position.y = ekf_ndt_odom.pose.pose.position.y;
			lcl5_.pose.pose.position.z = 0.0;
			lcl5_.pose.pose.orientation = ekf_ndt_odom.pose.pose.orientation;
			pub_lcl5.publish(lcl5_);
			
			// vis_ekf_ndt.header.stamp = ros::Time::now();
			// vis_ekf_ndt.pose.pose.position.x = ekf_ndt_odom.pose.pose.position.x;
			// vis_ekf_ndt.pose.pose.position.y = ekf_ndt_odom.pose.pose.position.y;
			// vis_ekf_ndt.pose.pose.position.z = 0.0;
			// vis_ekf_ndt.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ekf_ndt_odom.pose.pose.orientation.z);
			// pub_vis_ekf_ndt.publish(vis_ekf_ndt);
			
			transform.setOrigin( tf::Vector3(lcl5_.pose.pose.position.x, lcl5_.pose.pose.position.y, 0.0) );
			// transform.setOrigin( tf::Vector3(ekf_odom.pose.pose.position.x, ekf_odom.pose.pose.position.y, 0.0) );
			// transform.setOrigin( tf::Vector3(ndt_odom.pose.pose.position.x, ndt_odom.pose.pose.position.y, 0.0) );
			// transform.setOrigin( tf::Vector3(lcl2_.pose.pose.position.x, lcl2_.pose.pose.position.y, 0.0) );
			// transform.setOrigin( tf::Vector3(lcl_.pose.pose.position.x, lcl_.pose.pose.position.y, 0.0) );
			tf::Quaternion q;
			q = tf::Quaternion(lcl5_.pose.pose.orientation.x, lcl5_.pose.pose.orientation.y, lcl5_.pose.pose.orientation.z, lcl5_.pose.pose.orientation.w);
			// q = tf::Quaternion(ekf_odom.pose.pose.orientation.x, ekf_odom.pose.pose.orientation.y, ekf_odom.pose.pose.orientation.z, ekf_odom.pose.pose.orientation.w);
			// q = tf::Quaternion(ndt_odom.pose.pose.orientation.x, ndt_odom.pose.pose.orientation.y, ndt_odom.pose.pose.orientation.z, ndt_odom.pose.pose.orientation.w);
			// q.setRPY(0, 0, ekf_ndt_odom.pose.pose.orientation.z);
			// q.setRPY(0, 0, lcl2_.pose.pose.orientation.z);
			// q.setRPY(0, 0, lcl_.pose.pose.orientation.z);

			//transform.setOrigin( tf::Vector3(lcl[2].x, lcl[2].y, 0.0) );
			//tf::Quaternion q;
			//q.setRPY(0, 0, lcl[2].yaw);
			
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));
		
			cnt ++;
		}
		roop.sleep();
		ros::spinOnce();
	}
//	fclose(fp_1);
	fclose(fp_2);
	return 0;

}
