#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <time.h>


using namespace std;


#define Width 10
#define Height 10
#define Resolution 0.25

#define Expand_radius 0.15

string PARENT_FRAME;
string CHILD_FRAME;
vector<double> DoF;


nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGrid input_grid_map;

vector< vector<int> > global_map_2d;
vector< vector<int> > base_input_map_2d;


bool sub_global_map = false;
bool sub_other_agents = false;

enum cost{FREE=0, LETHAL=100};

class MapIndex{
private:
	MapIndex();
public:
	MapIndex(int _i, int _j):i(_i),j(_j){	}
	MapIndex(const MapIndex& id):i(id.i),j(id.j){	}
	
	int i,j;
};
bool operator==(const MapIndex& lhs, const MapIndex& rhs){
	return ((lhs.i==rhs.i) && (lhs.j==rhs.j));
}

bool operator<(const MapIndex& lhs, const MapIndex& rhs){
	return ((1000*lhs.i+lhs.j) < (1000*rhs.i+rhs.j));
}

class ExpandMap{
private:

	list<MapIndex> expanded_circle;
	
	ExpandMap(const ExpandMap&);
	
public:
	ExpandMap(float _radius, float resolution);
	
	void expandObstacle(const nav_msgs::OccupancyGrid& map_in);
	
	nav_msgs::OccupancyGrid local_map;
};

// midpoint circle algorithm
ExpandMap::ExpandMap(float _radius, float resolution)
{
	int radius=round(_radius/resolution);

	int f=1-radius;
	int ddF_x=1;
	int ddF_y=-2*radius;
	int x=0;
	int y=radius;
	
	expanded_circle.push_back(MapIndex(0,radius));
	expanded_circle.push_back(MapIndex(0,-radius));
	expanded_circle.push_back(MapIndex(radius,0));
	expanded_circle.push_back(MapIndex(-radius,0));
	
	// draw circle line on grid map
	while(x<y){
		if(f>=0){
			y--;
			ddF_y+=2;
			f+=ddF_y;
		}
	
		x++;
		ddF_x+=2;
		f+=ddF_x;
		
		expanded_circle.push_back(MapIndex(x , y));
		expanded_circle.push_back(MapIndex(-x, y));
		expanded_circle.push_back(MapIndex(x ,-y));
		expanded_circle.push_back(MapIndex(-x,-y));
		if(x!=y){
		expanded_circle.push_back(MapIndex( y, x));
		expanded_circle.push_back(MapIndex(-y, x));
		expanded_circle.push_back(MapIndex( y,-x));
		expanded_circle.push_back(MapIndex(-y,-x));
		}
	}


	// delete several overlap grids
	expanded_circle.sort();
	expanded_circle.unique();
	
}

void ExpandMap::expandObstacle(const nav_msgs::OccupancyGrid& map_in)
{
	local_map=map_in;
	
	vector<int8_t>::iterator itr;
	for(itr=local_map.data.begin(); itr!=local_map.data.end(); itr++){
		*itr=FREE;
	}
	
	
	for(int xi=0; xi<(int)map_in.info.height; xi++){
		for(int yi=0; yi<(int)map_in.info.width; yi++){
			// if the cell is LETHAL
			if(map_in.data[xi+map_in.info.width*yi]!=FREE){
				local_map.data[xi+map_in.info.width*yi] = map_in.data[xi+map_in.info.width*yi];
				// expand the LETHAL cells with respect to the circle radius
				list<MapIndex>::iterator litr;
				for(litr=expanded_circle.begin(); litr!=expanded_circle.end(); litr++){
					int x=xi+litr->i, y=yi+litr->j;
					if(x>=0 && x<(int)local_map.info.height && y>=0 && y<(int)local_map.info.width
						&& map_in.data[xi+map_in.info.width*yi]
							>local_map.data[x+map_in.info.width*y]){
						local_map.data[x+map_in.info.width*y] = 
							map_in.data[xi+map_in.info.width*yi];
					}
				}
			}
		}
	}
}

void print_param(){
	printf("Parent frame : %s\n", PARENT_FRAME.c_str());
	printf("Child frame : %s\n", CHILD_FRAME.c_str());
	printf("6Dof : (%f, %f, %f, %f, %f, %f)\n", DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}

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

vector<int> reshape_1dim(vector< vector<int> > input_array)
{
	size_t rows = input_array.size();
	size_t cols = input_array[0].size();
	vector<int> output_array(rows*cols, -1);
	for(size_t i=0; i<rows; i++){
		for(size_t j=0; j<cols; j++){
			output_array[j+i*cols] = input_array[i][j];
		}
	}
	// cout<<"grid_map_1dim : "<<endl;
	// view_array(output_array);
	return output_array;
}

vector<int> continuous2discreate(double x, double y, nav_msgs::OccupancyGrid map, bool origin)
{
	int p, q;
	if(origin){
		double resolution = map.info.resolution;
		double origin_x = -1.0 * map.info.origin.position.x;
		double origin_y = -1.0 * map.info.origin.position.y;
		// cout<<"width : "<<width<<endl;
		// cout<<"height : "<<height<<endl;
		// cout<<"resolution : "<<resolution<<endl;

		// cout<<"x : "<<x<<endl;
		// cout<<"y : "<<y<<endl;
		p = (x + origin_x) / resolution;
		// cout<<"p : " <<p<<endl;
		q = (y + origin_y) / resolution;
		// cout<<"q : " <<q<<endl;
	}
	else{
		double resolution = map.info.resolution;
		// cout<<"width : "<<width<<endl;
		// cout<<"height : "<<height<<endl;
		// cout<<"resolution : "<<resolution<<endl;
		
		// cout<<"x : "<<x<<endl;
		// cout<<"y : "<<y<<endl;
		p = x / resolution;
		// cout<<"p : " <<p<<endl;
		q = y / resolution;
		// cout<<"q : " <<q<<endl;
	}

	vector<int> output;
	output.resize(2);
	output[0] = p;
	output[1] = q;
	
	return output;
}

void set_grid_map(nav_msgs::OccupancyGrid &map)
{
	map.header.frame_id = CHILD_FRAME;
	map.header.stamp = global_map.header.stamp;
	map.info.resolution = Resolution;
	map.info.width = Width / Resolution;
	map.info.height = Height / Resolution;

	map.info.origin.position.x = -Width / 2;
	map.info.origin.position.y = -Height / 2;

	map.data.resize(map.info.width*map.info.height);
}

void set_grid_data(nav_msgs::OccupancyGrid &map, vector<int> map_1d)
{
	size_t map_size = map.data.size();
	for(size_t i=0; i<map_size; i++){
		map.data[i] = map_1d[i];
	}
}


vector<int> get_discreate_extract_range(vector<int> center,int width, int height, 
										vector<int> offset)
{
	vector<int> half_extract_range = continuous2discreate(width/2, height/2, global_map, false);
	cout<<"half_extract_range : "<<endl;
	view_array(half_extract_range);
	int min_x = (center[0] + offset[0]) - half_extract_range[0];
	int min_y = (center[1] + offset[1]) - half_extract_range[1];
	int max_x = (center[0] + offset[0]) + half_extract_range[0];
	int max_y = (center[1] + offset[1]) + half_extract_range[1];
	vector<int> discreate_extract_range{min_x, min_y, max_x, max_y};
	cout<<"discreate_extract_range : "<<endl;
	view_array(discreate_extract_range);

	return discreate_extract_range;
}

void extract_grid_2dim(vector< vector<int> > input, 
					   vector< vector<int> > &output, 
					   vector<int> extract_range)
{
	for(int y=extract_range[1]; y<extract_range[3]; y++){
		vector<int> col_vector;
		for(int x=extract_range[0]; x<extract_range[2]; x++){
			col_vector.push_back(input[y][x]);
		}
		output.push_back(col_vector);		
	}
	// view_gridmap(output);
}

void extract_base_input_grid_map(vector< vector<int> > &base_grid_map)
{
	vector<int> global_center = continuous2discreate(-1.0*global_map.info.origin.position.x, 
													 -1.0*global_map.info.origin.position.y, 
													 global_map, false);
	double offset_x = DoF[0];
	double offset_y = DoF[1];
	vector<int> discreate_offset = continuous2discreate(offset_x, offset_y, global_map, false);

	view_array(global_center);
	vector<int> extract_range 
		= get_discreate_extract_range(global_center, Width, Height, discreate_offset);

	extract_grid_2dim(global_map_2d, base_grid_map, extract_range);
	// view_gridmap(base_grid_map);
}

void set_grid_map_with_other_agent(vector< vector<int> > &grid_map, vector<int> agent_position)
{
	grid_map[agent_position[1]][agent_position[0]] = 100;
	grid_map[agent_position[1]+1][agent_position[0]] = 100;
	grid_map[agent_position[1]][agent_position[0]+1] = 100;
	grid_map[agent_position[1]][agent_position[0]-1] = 100;
	grid_map[agent_position[1]-1][agent_position[0]] = 100;
	grid_map[agent_position[1]+1][agent_position[0]+1] = 100;
	grid_map[agent_position[1]-1][agent_position[0]+1] = 100;
	// grid_map[agent_position[1]+1][agent_position[0]-1] = 100;
	// grid_map[agent_position[1]-1][agent_position[0]-1] = 100;
}

void globalMapCallback(nav_msgs::OccupancyGrid msg)
{
	sub_global_map = true;

	global_map = msg;
	cout<<"global_map.info.width : "<<global_map.info.width<<endl;
	cout<<"global_map.info.height : "<<global_map.info.height<<endl;
	cout<<"global_map.info.resolution : "<<global_map.info.resolution<<endl;
	cout<<"global_map.info.origin.position.x : "<<global_map.info.origin.position.x<<endl;
	cout<<"global_map.info.origin.position.y : "<<global_map.info.origin.position.y<<endl;

	global_map_2d = reshape_2dim(global_map.data, global_map.info.height, global_map.info.width);
	extract_base_input_grid_map(base_input_map_2d);
	cout<<"base_input_map_2d : "<<endl;
	view_gridmap(base_input_map_2d);

	set_grid_map(input_grid_map);
}

void otherAgentsVelocityCallback(visualization_msgs::MarkerArray msg)
{
	sub_other_agents = true;

	size_t num_other_agents = msg.markers.size();
	cout<<"num_other_agents : "<<num_other_agents<<endl;

	clock_t start = clock();
	if(sub_global_map){
		if(num_other_agents != 0){
			vector< vector<int> > input_map_2d = base_input_map_2d;
			for(size_t i=0; i<num_other_agents; i++){
				vector<int> discreate_position 
					= continuous2discreate(msg.markers[i].pose.position.x, 
										   msg.markers[i].pose.position.y, 
										   input_grid_map, true);
				cout<<"agent_id : "<<i<<endl;
				view_array(discreate_position);
				input_map_2d[discreate_position[1]][discreate_position[0]] = 100;
				set_grid_map_with_other_agent(input_map_2d, discreate_position);
			}
			// view_gridmap(input_map_2d);
			vector<int> input_map_1d = reshape_1dim(input_map_2d);
			set_grid_data(input_grid_map, input_map_1d);
		}
	}

	clock_t end = clock();
	cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "extract_grid_map");
	ros::NodeHandle n;

	n.getParam("/map2input_map/parent_frame", PARENT_FRAME);
	n.getParam("/map2input_map/child_frame", CHILD_FRAME);
	n.getParam("/map2input_map/6DoF", DoF);

	print_param();

	ros::Subscriber global_map_sub = n.subscribe("/map", 1, globalMapCallback);
	ros::Subscriber other_agents_velocity_sub 
		= n.subscribe("/other_agents_velocity", 1, otherAgentsVelocityCallback);

	ros::Publisher input_grid_map_pub 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map", 1);
	ros::Publisher input_grid_map_expand_pub 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/expand", 1);

	cout<<"Here we go!!"<<endl;

	ExpandMap ex_map(Expand_radius, Resolution);

	ros::Rate loop_rate(20);

	while(ros::ok()){
		if(sub_other_agents && sub_global_map){
			ex_map.expandObstacle(input_grid_map);
			input_grid_map_pub.publish(input_grid_map);
			input_grid_map_expand_pub.publish(ex_map.local_map);
		}
		else{
			if(!sub_global_map){
				printf("Wait for \"/map\" topic!!\n");
			}
			if(!sub_other_agents){
				printf("Wait for \"/ather_agents_velocity\" topic!!\n");
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
