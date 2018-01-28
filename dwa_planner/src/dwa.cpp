#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <knm_tiny_msgs/Velocity.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <time.h>

#ifdef _OEPNMP
#include <omp.h>
#endif

using namespace std;

#define COST_OBS 10.0
#define COST_VEL 1.0
#define COST_HEAD 1.0


double MAX_VEL;
double MIN_VEL;
double MAX_ROT_VEL;
double ACC_LIM_TRANS;
double ACC_LIM_ROT;
double VEL_SAMPLES;
double ROT_VEL_SAMPLES;
double SIM_TIME;

visualization_msgs::MarkerArray path_candidate;
visualization_msgs::Marker selected_path;

nav_msgs::OccupancyGrid local_map;
nav_msgs::Odometry current_state;
geometry_msgs::Point next_target;

vector<geometry_msgs::Point> obs_position;


ros::Publisher vis_target_pub;


bool sub_local_map = false;
bool sub_lcl = false;
bool sub_next_target = false;

vector<double> get_DynamicWindow(nav_msgs::Odometry state, double dt)
{
	vector<double> Vs{MIN_VEL, MAX_VEL, -MAX_ROT_VEL, MAX_ROT_VEL};
	cout<<"**********************"<<endl;
	for(size_t i=0; i<Vs.size();i++){
		cout<<"Vs["<<i<<"] : "<<Vs[i]<<endl;
	}
	double linear = state.twist.twist.linear.x;
	double angular = state.twist.twist.angular.z;
	vector<double> Vd{linear-ACC_LIM_TRANS*dt, linear+ACC_LIM_TRANS*dt, 
					  angular-ACC_LIM_ROT*dt, angular+ACC_LIM_ROT*dt};
	cout<<"++++++++++++++++++++++"<<endl;
	for(size_t i=0; i<Vd.size();i++){
		cout<<"Vd["<<i<<"] : "<<Vd[i]<<endl;
	}
	vector<double> Vr = Vs;
	size_t Vr_size = Vr.size();
	for(size_t i=0; i<Vr_size; i++){
		if(i%2==0){
			if(Vd[i] > Vr[i]){
				Vr[i] = Vd[i];
			}
		}
		else{
			if(Vd[i] < Vr[i]){
				Vr[i] = Vd[i];
			}	
		}
	}
	cout<<"--------------------"<<endl;
	for(size_t i=0; i<Vr.size();i++){
		cout<<"Vr["<<i<<"] : "<<Vr[i]<<endl;
	}
	return Vr;
}

vector<double> get_sample_resolution(vector<double> Vr, 
									 int num_samples_vel, int num_samples_rot_vel)
{
	double diff_vel = Vr[1] - Vr[0];
	cout<<"diff_vel : "<<diff_vel<<endl;
	double diff_rot_vel = Vr[3] - Vr[2];
	cout<<"diff_rot_vel : "<<diff_rot_vel<<endl;
	double res_vel = fabs(diff_vel / num_samples_vel);
	double res_rot_vel = fabs(diff_rot_vel / num_samples_rot_vel);
	cout<<"res_vel : "<<res_vel<<endl;
	cout<<"res_rot_vel : "<<res_rot_vel<<endl;
	vector<double> sample_resolutions{res_vel, res_rot_vel};

	return sample_resolutions;
}

void set_vis_traj(vector<geometry_msgs::PoseStamped> traj, 
				  visualization_msgs::Marker &marker, int id)
{
	size_t traj_size = traj.size();
	marker.header = traj[traj_size-1].header;
	marker.id = id;
	ostringstream ss;
	ss << id;
	marker.ns = "trajectory_" + ss.str();
	// cout<<"marker.ns : "<<marker.ns<<endl;

	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.005;

	marker.color.r = 0.5;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	// marker.lifetime = ros::Duration(0.1);
	marker.lifetime = ros::Duration(0.025);
	
	for(size_t i=0; i<traj_size; i++){
		marker.points.push_back(traj[i].pose.position);
	}
}

visualization_msgs::Marker get_selected_path(visualization_msgs::MarkerArray candidate, int index){
	visualization_msgs::Marker selected_path;
	selected_path = candidate.markers[index];
	selected_path.color.r = 1.0;
	selected_path.color.g = 0.0;
	selected_path.color.b = 0.0;
	selected_path.color.a = 1.0;
	selected_path.scale.x = 0.01;

	size_t path_size = selected_path.points.size();
	for(size_t i=0; i<path_size; i++){
		selected_path.points[i].z = 0.01;
	}
	return selected_path;
}

geometry_msgs::PoseStamped set_robot_pose(double x, double y, double yaw)
{
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id = "/velodyne";
	robot_pose.header.stamp = ros::Time::now();

	robot_pose.pose.position.x = x;
	robot_pose.pose.position.y = y;
	robot_pose.pose.position.z = 0.0;

	robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	// cout<<"robot_pose : "<<robot_pose<<endl;
	return robot_pose;
	
}

geometry_msgs::PoseStamped move(geometry_msgs::PoseStamped robot_pose, 
								double linear, double angular, double dt)
{
	double next_yaw = tf::getYaw(robot_pose.pose.orientation) + angular*dt;
	// cout<<"next_yaw : "<<next_yaw<<endl;
	double next_x = robot_pose.pose.position.x + linear*cos(next_yaw)*dt;
	// cout<<"next_x : "<<next_x<<endl;
	double next_y = robot_pose.pose.position.y + linear*sin(next_yaw)*dt;
	// cout<<"next_y : "<<next_y<<endl;
	geometry_msgs::PoseStamped next_pose;
	next_pose = set_robot_pose(next_x, next_y, next_yaw);
	
	return next_pose;
}

vector<geometry_msgs::PoseStamped> get_future_trajectory(double linear, double angular, double sim_time, double dt)
{
	vector<geometry_msgs::PoseStamped> trajectory;
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = set_robot_pose(0.0, 0.0, 0.0);
	trajectory.push_back(robot_pose);
	double time = 0.0;
	while(time <= sim_time){
		time += dt;
		robot_pose = move(robot_pose, linear, angular, dt);
		trajectory.push_back(robot_pose);
	}

	// for(size_t i=0; i<trajectory.size(); i++){
		// cout<<"trajectory["<<i<<"] : "<<trajectory[i]<<endl;
	// }
	
	return trajectory;
}

double calc_dist(geometry_msgs::Point a, geometry_msgs::Point b)
{
	return sqrt(pow((a.x-b.x), 2) + pow((a.y-b.y), 2));
}

double check_nearest_obs_dist(vector<geometry_msgs::PoseStamped> traj, 
							vector<geometry_msgs::Point> obs_list)
{
	geometry_msgs::PoseStamped final_pose;
	vector<geometry_msgs::PoseStamped>::iterator itr = traj.end()-1;
	final_pose = *itr;
	// cout<<"final_pose : "<<final_pose<<endl;
	double min_dist = 100.0;
	size_t obs_list_size = obs_list.size();
	if(obs_list_size != 0){
		min_dist = calc_dist(obs_list[0], final_pose.pose.position);
		for(size_t i=1; i<obs_list_size; i++){
			double tmp_dist = calc_dist(obs_list[i], final_pose.pose.position);
			// cout<<"dist : "<<tmp_dist<<endl;
			if(tmp_dist<min_dist){
				min_dist = tmp_dist;
			}
		}
	}
	// cout<<"min_dist : "<<min_dist<<endl;

	return min_dist;
}

double check_goal_heading(vector<geometry_msgs::PoseStamped> traj, geometry_msgs::Point target)
{
	geometry_msgs::PoseStamped final_pose;
	vector<geometry_msgs::PoseStamped>::iterator itr = traj.end()-1;
	final_pose = *itr;
	// cout<<"final_pose : "<<final_pose<<endl;
	double final_yaw = tf::getYaw(final_pose.pose.orientation);
	// cout<<"final_yaw : "<<final_yaw<<endl;
	// cout<<"target : "<<target<<endl;
	double goal_theta = atan2(target.y-final_pose.pose.position.y, 
							  target.x-final_pose.pose.position.x);
	// cout<<"goal_theta : "<<goal_theta<<endl;
	double target_theta;
	if(goal_theta > final_yaw){
		target_theta = goal_theta - final_yaw;
	}
	else{
		target_theta = final_yaw - goal_theta;
	}
	double eval_heading = M_PI - target_theta;
	// cout<<"eval_heading : "<<eval_heading<<endl;
	return eval_heading;
}


vector<double> evaluation_trajectories(vector<double> Vr, vector<double> sample_resolutions, double dt)
{
	path_candidate.markers.clear();
	vector< vector<double> > path_and_eval_list;
	int i = 0;
	for(double linear=Vr[0]; linear<=Vr[1]; linear+=sample_resolutions[0]){
		for(double angular=Vr[2]; angular<Vr[3]; angular+=sample_resolutions[1]){
			// cout<<"============== i : "<<i<<" ============ "<<endl;
			// cout<<"linear : "<<linear<<endl;
			// cout<<"angular : "<<angular<<endl;
			vector<geometry_msgs::PoseStamped> trajectory;
			trajectory = get_future_trajectory(linear, angular, SIM_TIME, dt);
			double eval_obs_dist = check_nearest_obs_dist(trajectory, obs_position);
			// cout<<"eval_obs_dist : "<<eval_obs_dist<<endl;
			double eval_vel = fabs(linear);
			// cout<<"eval_vel : "<<eval_vel<<endl;
			double eval_heading = check_goal_heading(trajectory, next_target);
			// cout<<"eval_heading : "<<eval_heading<<endl;

			vector<double> path_and_eval{linear, angular, eval_obs_dist, eval_vel, eval_heading};
			path_and_eval_list.push_back(path_and_eval);
			
			visualization_msgs::Marker vis_traj;
			set_vis_traj(trajectory, vis_traj, i);
			path_candidate.markers.push_back(vis_traj);
			i++;
		}
	}

	double max_total_eval = COST_OBS*path_and_eval_list[0][2] 
						  + COST_VEL*path_and_eval_list[0][3] 
						  + COST_HEAD*path_and_eval_list[0][4]; 
	size_t max_eval_index = 0;
	size_t path_and_eval_list_size = path_and_eval_list.size();
	for(size_t i=1; i<path_and_eval_list_size; i++){
		// cout<<"============== i : "<<i<<" ============ "<<endl;
		double tmp_total_eval = COST_OBS*path_and_eval_list[i][2] 
							  + COST_VEL*path_and_eval_list[i][3] 
							  + COST_HEAD*path_and_eval_list[i][4]; 
		// cout<<"tmp_total_eval : "<<tmp_total_eval<<endl;
		if(tmp_total_eval > max_total_eval){
			max_total_eval = tmp_total_eval;
			max_eval_index = i;
		}
	}
	// cout<<"max_total_eval : "<<max_total_eval<<endl;
	// cout<<"max_eval_index : "<<max_eval_index<<endl;

	selected_path = get_selected_path(path_candidate, max_eval_index);
	double selected_linear = path_and_eval_list[max_eval_index][0];
	double selected_angular = path_and_eval_list[max_eval_index][1];
	
	vector<double> selected_velocity_vector{selected_linear, selected_angular};

	return selected_velocity_vector;
}


vector<int> find_index(const std::vector<int> &v, int val)
{
	vector<int> find_index;
	size_t v_size = v.size();
	for(size_t i=0; i!=v_size; ++i){
		if( v[i] == val ){
			find_index.push_back(i);
		}
	}
	return find_index;
}

vector<geometry_msgs::Point> get_continuous_obs_position(nav_msgs::OccupancyGrid map)
{
	vector<int> grid_data(map.info.height*map.info.width);
	// cout<<map<<endl;
	size_t map_data_size = map.data.size();
	for(size_t i=0; i<map_data_size; i++){
		grid_data[i] = map.data[i];
	}
	// cout<<"grid_data : "<<grid_data.size()<<endl;
	vector<int> obs_index;
	obs_index = find_index(grid_data, 100);
	// for(size_t i=0; i<obs_index.size(); i++){
		// cout<<"obs_index["<<i<<"] : "<<obs_index[i]<<endl;
	// }
	size_t obs_index_size = obs_index.size();
	// cout<<"obs_index_size : "<<(int)obs_index_size<<endl;
	vector<geometry_msgs::Point> continuous_obs_position(obs_index_size);
	for(size_t i=0; i<obs_index_size; i++){
		double x = (obs_index[i] % map.info.width) * map.info.resolution + map.info.origin.position.x;
		// cout<<"x : "<<x<<endl;
		double y = int(obs_index[i] / map.info.width) * map.info.resolution + map.info.origin.position.y;
		// cout<<"y : "<<y<<endl;
		geometry_msgs::Point obs_position;
		obs_position.x = x;
		obs_position.y = y;
		obs_position.z = 0.0;

		continuous_obs_position.push_back(obs_position);
	}

	return continuous_obs_position;
}

void localMapCallback(nav_msgs::OccupancyGrid msg)
{
	local_map = msg;
	// cout<<"Subscribe local_map!!"<<endl;
	obs_position = get_continuous_obs_position(msg);
	sub_local_map = true;
}

void lclCallback(nav_msgs::Odometry msg)
{
	current_state = msg;
	// cout<<"Subscribe lcl!!"<<endl;
	sub_lcl = true;
}

void set_target_marker(geometry_msgs::Point target, visualization_msgs::Marker &marker)
{
	marker.header.frame_id = "/velodyne";
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.ns = "vin_target";

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;

	marker.color.r = 0.7;
	marker.color.g = 0.0;
	marker.color.b = 0.7;
	marker.color.a = 1.0;

	// marker.lifetime = ros::Duration(0.1);
	marker.lifetime = ros::Duration(0.025);

	marker.pose.position = target;
}

void vinNextTargetCallback(std_msgs::Int32MultiArray msg)
{
	double res = local_map.info.resolution * float(local_map.info.width) / float(msg.layout.dim[0].size);
	// cout<<"local_map.info.resolution : "<<local_map.info.resolution<<endl;
	// cout<<"local_map.info.width : "<<local_map.info.width<<endl;
	// cout<<"res : "<<res<<endl;
	double x = msg.data[1] * res + local_map.info.origin.position.x;
	double y = msg.data[0] * res + local_map.info.origin.position.y;
	
	next_target.x = x;
	next_target.y = y;
	next_target.z = 0.0;

	visualization_msgs::Marker vis_target;
	set_target_marker(next_target, vis_target);
	
	vis_target_pub.publish(vis_target);
	// cout<<"next_target : "<<next_target<<endl;

	sub_next_target = true;
}

void print_param(){
	printf("max_velocity : %.4f\n", MAX_VEL);	
	printf("min_velocity : %.4f\n", MIN_VEL);	
	printf("max_rotation_velocity : %.4f\n", MAX_ROT_VEL);	
	printf("acceleraton_limit_translation : %.4f\n", ACC_LIM_TRANS);	
	printf("acceleraton_limit_rotation : %.4f\n", ACC_LIM_ROT);	
	printf("velocity_samples : %.4f\n", VEL_SAMPLES);	
	printf("rotation_velocity_samples : %.4f\n", ROT_VEL_SAMPLES);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dwa");
	ros::NodeHandle n;

	n.getParam("/dwa/max_velocity", MAX_VEL);
	n.getParam("/dwa/min_velocity", MIN_VEL);
	n.getParam("/dwa/max_rotation_velocity", MAX_ROT_VEL);
	n.getParam("/dwa/acceleraton_limit_translation", ACC_LIM_TRANS);
	n.getParam("/dwa/acceleraton_limit_rotation", ACC_LIM_ROT);
	n.getParam("/dwa/velocity_samples", VEL_SAMPLES);
	n.getParam("/dwa/rotation_velocity_samples", ROT_VEL_SAMPLES);
	n.getParam("/dwa/simulation_time", SIM_TIME);
	print_param();


	ros::Subscriber local_map_sub = n.subscribe("/local_map_real", 1, localMapCallback);
	ros::Subscriber lcl_sub = n.subscribe("/lcl5", 1, lclCallback);
	ros::Subscriber vin_next_targe_sub = n.subscribe("/vin/next_target", 1, vinNextTargetCallback);

	ros::Publisher cmd_vel_pub = n.advertise<knm_tiny_msgs::Velocity>("/control_command", 1);
	ros::Publisher vis_path_selected_pub = n.advertise<visualization_msgs::Marker>("/vis_path/selected", 1);
	ros::Publisher vis_path_candidate_pub = n.advertise<visualization_msgs::MarkerArray>("/vis_path/candidate", 1);
	vis_target_pub = n.advertise<visualization_msgs::Marker>("/vin/next_target/vis", 1);

	cout<<"Here we go!!"<<endl;

	ros::Rate loop_rate(40);
	// double dt = 1.0 / 40.0;
	double dt = 0.1;

	while(ros::ok()){
		// cout<<"**********************"<<endl;
		if(sub_local_map && sub_lcl && sub_next_target){
			clock_t start = clock();
			vector<double> Vr;
			Vr = get_DynamicWindow(current_state, dt);
			vector<double> sample_resolutions;
			sample_resolutions = get_sample_resolution(Vr, VEL_SAMPLES, ROT_VEL_SAMPLES);
			vector<geometry_msgs::PoseStamped> trajectory;
			// trajectory = get_future_trajectory(Vr[0], Vr[2], SIM_TIME, dt);
			// visualization_msgs::Marker vis_traj;
			// set_vis_traj(trajectory, vis_traj, 0);
			vector<double> velocity_vector;
			velocity_vector = evaluation_trajectories(Vr, sample_resolutions, dt);
			cout<<"linear : "<<velocity_vector[0]<<endl;
			cout<<"angular : "<<velocity_vector[1]<<endl;

			knm_tiny_msgs::Velocity control_command;
			control_command.op_linear = velocity_vector[0];
			control_command.op_angular = -1.0*velocity_vector[1];
			cmd_vel_pub.publish(control_command);

			// vis_path_single_pub.publish(vis_traj);
			vis_path_candidate_pub.publish(path_candidate);
			vis_path_selected_pub.publish(selected_path);
			clock_t end = clock();
			cout<<"duration : "<<(double)(end - start) / CLOCKS_PER_SEC<<"[sec]"<<endl;

			// sub_local_map = false;
			// sub_lcl = false;
			// sub_next_target = false;
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
    
	return 0;
}
