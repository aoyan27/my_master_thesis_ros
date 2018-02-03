#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

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
	ros::init(argc, argv, "set_tf_map2world");
	ros::NodeHandle n;
	
	n.getParam("/map2world/parent_frame", PARENT_FRAME);
	n.getParam("/map2world/child_frame", CHILD_FRAME);
	n.getParam("/map2world/6DoF", DoF);

	print_param();

	cout<<"Here we go!!"<<endl;

	tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(DoF[0], DoF[1], DoF[2]));

	tf::Quaternion q;
	q.setRPY(DoF[3], DoF[4], DoF[5]);
	transform.setRotation(q);

	ros::Rate loop_rate(10);
	while(ros::ok()){
		printf("Send tf!!");
		printf("Parent frame : %s\n", PARENT_FRAME.c_str());
		printf("Child frame : %s\n", CHILD_FRAME.c_str());
		printf("6Dof : (%f, %f, %f, %f, %f, %f)\n", DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/world"));

		loop_rate.sleep();
	}

	return 0;
}
