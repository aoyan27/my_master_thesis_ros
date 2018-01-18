#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <stdio.h>
#include <algorithm>
#include <time.h>

#ifdef _OEPNMP
#include <omp.h>
#endif

using namespace std;

#define NUM_ACTION 8
#define COST 1
#define LETHAL 100


nav_msgs::OccupancyGrid global_map;
geometry_msgs::PoseStamped target_pose;
nav_msgs::Odometry lcl;
	
bool sub_global_map = false;
bool sub_target_pose = false;
bool sub_lcl = false;

vector< vector<int> > heuristic;



void view_gridmap(vector< vector<int> > array)
{	
	size_t array_size1 = array.size(); 
	// cout<<"array_size1 : "<<array_size1<<endl;
	size_t array_size2 = array[0].size(); 
	// cout<<"array_size2 : "<<array_size2<<endl;
	for(size_t i=0; i<array_size1; i++){
		for(size_t j=0; j<array_size2; j++){
			printf("%3d, ", array[i][j]);
		}
		cout<<endl;
	}
}

void view_array(vector<int> array)
{	
	size_t array_size1 = array.size(); 
	// cout<<"array_size1 : "<<array_size1<<endl;
	for(size_t i=0; i<array_size1; i++){
		printf("%3d, ", array[i]);
	}
	cout<<endl;
}

vector< vector<int> > reshape_2dim(vector<signed char> input_array, int rows, int cols)
{
	vector< vector<int> > output_array;
	for(int i=0; i<rows; i++){
		vector<int> a(cols, 0);
		for(int j=0; j<cols; j++){
			a[j] = input_array[j+i*cols];
		}
		output_array.push_back(a);
	}
	// cout<<"grid_map : "<<endl;
	// view_gridmap(output_array);
	return output_array;
}

vector< vector<int> > set_2dim_array(int rows, int cols, int init_num)
{
	vector< vector<int> > output;
	output.resize(rows);
	for(int i=0; i<rows; i++){
		vector<int> a(cols, init_num);
		output[i] = a;
	}
	return output;
	
}

vector<int> continuous2discreate(double x, double y)
{
	double resolution = global_map.info.resolution;
	double origin_x = -1.0 * global_map.info.origin.position.x;
	double origin_y = -1.0 * global_map.info.origin.position.y;
	// cout<<"width : "<<width<<endl;
	// cout<<"height : "<<height<<endl;
	// cout<<"resolution : "<<resolution<<endl;
	
	// cout<<"x : "<<x<<endl;
	// cout<<"y : "<<y<<endl;
	int p = (x + origin_x) / resolution;
	// cout<<"p : " <<p<<endl;
	int q = (y + origin_y) / resolution;
	// cout<<"q : " <<q<<endl;
	vector<int> output;
	output.resize(2);
	output[0] = p;
	output[1] = q;
	
	return output;
}

vector< vector<int> > create_heuristic(int rows, int cols, vector<int> target)
{
	int goal_x = target[0];
	int goal_y = target[1];
	vector< vector<int> > heuristic = set_2dim_array(rows, cols, 0);
	for(int y=0; y<rows; y++){
		for(int x=0; x<cols; x++){
			int diff_y = abs(goal_y - y);
			int diff_x = abs(goal_x - x);
			heuristic[y][x] = diff_y + diff_x;
		}
	}
	// cout<<"heuristic : "<<endl;;
	// view_gridmap(heuristic);
	return heuristic;

}

vector<int> pop_list(vector< vector<int> > &input_array)
{
	size_t end_index = input_array.size() - 1;
	// cout<<"end_index : "<<end_index<<endl;
	size_t array_size = input_array[0].size();
	vector<int> output_array;
	output_array.resize(array_size);
	for(size_t i=0; i<array_size; i++){
		output_array[i] = input_array[end_index][i];
		// cout<<input_array[end_index][i]<<endl;;
	}
	input_array.pop_back();
	return output_array;
}

bool move(vector< vector<int> > grid_map, int x, int y, 
		  int &next_x, int &next_y, int action, int reflect)
{
	bool collision = false;

	next_x = x;
	next_y = y;

	if(action==0){
		// right
		next_x = x + reflect*1;
	}
	else if(action==1){
		// down right
		next_x = x + reflect*1;
		next_y = y + reflect*1;
	}
	else if(action==2){
		// down
		next_y = y + reflect*1;
	}
	else if(action==3){
		// down left
		next_x = x - reflect*1;
		next_y = y + reflect*1;
	
	}
	else if(action==4){
		// left
		next_x = x - reflect*1;
	}
	else if(action==5){
		// upper left
		next_x = x - reflect*1;
		next_y = y - reflect*1;
	}
	else if(action==6){
		// up
		next_y = y - reflect*1;
	}
	else if(action==7){
		// upper right
		next_x = x + reflect*1;
		next_y = y - reflect*1;
	}
	else{
		next_x = x;
		next_y = y;
	}

	if(grid_map[next_y][next_x] == LETHAL){
		collision =true;
	}

	return collision;
}

bool a_star(nav_msgs::Odometry state, geometry_msgs::PoseStamped target, 
			vector< vector<int> > &state_list, vector<int> &shortest_action_list)
{
	if(sub_global_map && sub_target_pose && sub_lcl){
		// cout<<"========================"<<endl;
		clock_t start_time = clock();
		vector<signed char> grid_data = global_map.data;
		int rows = global_map.info.height;
		int cols = global_map.info.width;
		// cout<<"rows : "<<rows<<endl;
		// cout<<"cols : "<<cols<<endl;
		vector<int> discreate_target 
			= continuous2discreate(target.pose.position.x, target.pose.position.y);
		int goal_x = discreate_target[0];
		int goal_y = discreate_target[1];
		// cout<<"discreate_target : ";
		// view_array(discreate_target);

		vector< vector<int> > grid_map = reshape_2dim(grid_data, rows, cols);
		// cout<<"grid_map : "<<endl;
		// view_gridmap(grid_map);
		vector< vector<int> > open_list;
		vector< vector<int> > closed_list = set_2dim_array(rows, cols, 0);
		vector< vector<int> > expand_list = set_2dim_array(rows, cols, -1);
		vector< vector<int> > action_list_ = set_2dim_array(rows, cols, -1);

		
		vector<int> discreate_state;
		discreate_state 
			= continuous2discreate(state.pose.pose.position.x, state.pose.pose.position.y);
		int start_x = discreate_state[0];
		int start_y = discreate_state[1];
		int x = start_x;
		int y = start_y;

		int g = 0;
		int h = heuristic[y][x];
		int f = g + h;
		
		vector<int> tmp_open_list{f, g, h, x, y};
		open_list.push_back(tmp_open_list);
		closed_list[y][x] = 1;

		bool found = false;
		bool resign = false;

		int n = 0;


		while(!found && !resign){
			// cout<<"=========================="<<endl;
			// cout<<"n : "<<n<<endl;
			if(open_list.size() == 0 || n > 150){
				resign = true;
			}
			else{
				// cout<<"open_list : "<<endl;
				// view_gridmap(open_list);
				sort(open_list.begin(), open_list.end());
				// cout<<"open_list(sorted) : "<<endl;
				// view_gridmap(open_list);
				reverse(open_list.begin(), open_list.end());
				// cout<<"open_list(reversed) : "<<endl;
				// view_gridmap(open_list);
				vector<int> current = pop_list(open_list);
				// cout<<"current";
				// view_array(current);

				f = current[0];
				g = current[1];
				h = current[2];

				x = current[3];
				y = current[4];
				// cout<<"x : "<<x<<endl;
				// cout<<"y : "<<y<<endl;
				expand_list[y][x] = n;
				n++;

				if(x==goal_x && y==goal_y){
					cout<<"found!!"<<endl;
					found = true;
					break;
				}

				for(int a=0; a<NUM_ACTION; a++){
					// cout<<"--------------------"<<endl;
					// cout<<"a : "<<a<<endl;
					int next_x, next_y;
					bool collision = move(grid_map, x, y, next_x, next_y, a, 1);
					// cout<<"next_x : "<<next_x<<endl;
					// cout<<"next_y : "<<next_y<<endl;
					// cout<<"collision : "<<collision<<endl;
					// cout<<"!collision : "<<!collision<<endl;
					if(!collision && closed_list[next_y][next_x] == 0){
						int next_g = g + COST;
						int next_h = heuristic[next_y][next_x];
						int next_f = next_g + next_h;
						vector<int> tmp_next_open_list{next_f, next_g, next_h, next_x, next_y};
						// cout<<"tmp_next_open_list : ";
						// view_array(tmp_next_open_list);
						open_list.push_back(tmp_next_open_list);
						// cout<<"open_list : "<<endl;
						// view_gridmap(open_list);

						closed_list[next_y][next_x] = 1;
						action_list_[next_y][next_x] = a;
					}
				}
			}
		}
		// cout<<"closed_list_ : "<<endl;
		// view_gridmap(closed_list);
		// cout<<"action_list_ : "<<endl;
		// view_gridmap(action_list_);
		// cout<<"found : "<<found<<endl;
		
		if(found){
			int tmp_x = goal_x;
			int tmp_y = goal_y;
			// cout<<"tmp_x : "<<tmp_x<<endl;
			// cout<<"tmp_y : "<<tmp_y<<endl;
			vector<int> state{tmp_x, tmp_y};
			state_list.push_back(state);
			shortest_action_list.push_back(action_list_[tmp_y][tmp_x]);

			// cout<<"start_x : "<<start_x<<endl;
			// cout<<"start_y : "<<start_y<<endl;
			int challenge_times = 0;
			while(tmp_x!=start_x || tmp_y!=start_y){
				challenge_times++;
				if(challenge_times > 300){
					break;
				}
				int before_x, before_y;
				bool collision = move(grid_map ,tmp_x, tmp_y, 
									  before_x, before_y, action_list_[tmp_y][tmp_x], -1);
				// cout<<"before_x : "<<before_x<<endl;
				// cout<<"before_y : "<<before_y<<endl;
				if(!collision){
					vector<int> before_state{before_x, before_y};
					state_list.push_back(before_state);
					shortest_action_list.push_back(action_list_[tmp_y][tmp_x]);

					tmp_x = before_x;
					tmp_y = before_y;
				}
			}
			reverse(state_list.begin(), state_list.end());
			view_array(shortest_action_list);
			reverse(shortest_action_list.begin(), shortest_action_list.end());
			view_array(shortest_action_list);
		}

		clock_t end_time = clock();
		cout<<"duration = " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "sec."<<endl;
		return found;
	}
	else{
		cout<<"No global map!!"<<endl;
		return false;
	}
}

nav_msgs::Path set_trajectory(vector< vector<int> > state_list)
{
	nav_msgs::Path traj;

	double resolution = global_map.info.resolution;
	double origin_x = -1.0 * global_map.info.origin.position.x;
	double origin_y = -1.0 * global_map.info.origin.position.y;

	size_t state_list_size = state_list.size();
	for(size_t i=0; i<state_list_size; i++){
		geometry_msgs::PoseStamped tmp_pose;
		tmp_pose.pose.position.x = state_list[i][0]*resolution-origin_x;
		tmp_pose.pose.position.y = state_list[i][1]*resolution-origin_y;
		tmp_pose.pose.position.z = 0.5;

		traj.poses.push_back(tmp_pose);
	}
	traj.header.frame_id = global_map.header.frame_id;
	traj.header.stamp = ros::Time::now();

	return traj;
}

void globalMapCallback(nav_msgs::OccupancyGrid msg)
{
	global_map = msg;
	cout<<"Subscribe global_map!!"<<endl;
	sub_global_map = true;
}

void targetPoseCallback(geometry_msgs::PoseStamped msg)
{
	if(sub_global_map){
		if(target_pose.pose.position.x != msg.pose.position.x 
				|| target_pose.pose.position.y != msg.pose.position.y)
		{
			int rows = global_map.info.height;
			int cols = global_map.info.width;
			vector<int> discreate_target 
				= continuous2discreate(msg.pose.position.x, msg.pose.position.y);
			heuristic = create_heuristic(rows, cols, discreate_target);
			cout<<"new heuristic!!"<<endl;
		}
		target_pose = msg;
		cout<<"Subscribe target_pose!!"<<endl;
		sub_target_pose = true;
	}
}

void lclCallback(nav_msgs::Odometry msg)
{
	lcl = msg;
	cout<<"Subscribe lcl!!"<<endl;
	sub_lcl = true;
}


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "a_star_global");
	ros::NodeHandle n;

	ros::Subscriber global_map_sub = n.subscribe("/map", 1, globalMapCallback);
	// ros::Subscriber global_map_sub = n.subscribe("/local_map_real", 1, globalMapCallback);
	ros::Subscriber target_pose_sub = n.subscribe("/target/pose", 1, targetPoseCallback);
	ros::Subscriber lcl_sub = n.subscribe("/lcl5", 1, lclCallback);

	ros::Publisher global_path_pub = n.advertise<nav_msgs::Path>("/global_path", 1);


	cout<<"Here we go!!"<<endl;

	ros::Rate loop_rate(1);
	
	vector< vector<int> > state_list;
	vector<int> action_list;
	
	bool path_found = false;
	
	nav_msgs::Path global_path;
	global_path.header.frame_id = global_map.header.frame_id;

	while(ros::ok()){
		cout<<"***********************"<<endl;
		path_found = a_star(lcl, target_pose, state_list, action_list);
		if(path_found){
			cout<<"state_list : "<<endl;
			view_gridmap(state_list);
			cout<<"action_list : "<<endl;
			view_array(action_list);
			
			global_path = set_trajectory(state_list);
			// global_path_pub.publish(global_path);

			state_list.clear();
			action_list.clear();

		}
		global_path.header.stamp = ros::Time::now();
		global_path_pub.publish(global_path);

		loop_rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
