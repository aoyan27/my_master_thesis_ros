#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <stdio.h>

using namespace std;

string PARENT_FRAME;
string CHILD_FRAME;
vector<double> DoF;


void print_param(){
	printf("Parent frame : %s\n", PARENT_FRAME.c_str());
	printf("Child frame : %s\n", CHILD_FRAME.c_str());
	printf("6Dof : (%f, %f, %f, %f, %f, %f)\n", DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_tf_map2input_map");
	ros::NodeHandle n;
	
	n.getParam("/map2input_map/parent_frame", PARENT_FRAME);
	n.getParam("/map2input_map/child_frame", CHILD_FRAME);
	n.getParam("/map2input_map/6DoF", DoF);

	print_param();

	cout<<"Here we go!!"<<endl;

	tf2_ros::StaticTransformBroadcaster br;

	geometry_msgs::TransformStamped br_transform;

	geometry_msgs::Transform transform;
	transform.translation.x = DoF[0];
	transform.translation.y = DoF[1];
	transform.translation.z = DoF[2];
	transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(DoF[3], DoF[4], DoF[5]);

	std_msgs::Header header;
	header.frame_id = PARENT_FRAME;

	printf("Send tf!!");
	printf("Parent frame : %s\n", PARENT_FRAME.c_str());
	printf("Child frame : %s\n", CHILD_FRAME.c_str());
	printf("6Dof : (%f, %f, %f, %f, %f, %f)\n", DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);

	header.stamp = ros::Time::now();

	br_transform.header = header;
	br_transform.child_frame_id = CHILD_FRAME;
	br_transform.transform = transform;

	br.sendTransform(br_transform);

	ros::spin();

	return 0;
}
