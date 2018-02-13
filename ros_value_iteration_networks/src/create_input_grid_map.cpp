#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <time.h>


#define Width 10
#define Height 10
#define Resolution 0.25

#define INV_Resolution 1.0/Resolution
#define HALF_WIDTH Width/2
#define HALF_LENGTH Height/2

#define MAP_WIDTH Width/Resolution
#define MAP_LENGTH Height/Resolution
#define MAP_SIZE MAP_WIDTH*MAP_LENGTH

#define _FREE 0
#define _UNEXPLORE 50
#define _LETHAL 100

#define Expand_radius 0.25


using namespace std;
using namespace Eigen;


nav_msgs::Odometry lcl;

nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGrid static_input_map;
nav_msgs::OccupancyGrid real_input_map;
nav_msgs::OccupancyGrid input_map;

vector< vector<int> > global_map_2d;
vector< vector<int> > static_input_map_2d;
vector< vector<int> > real_input_map_2d;
vector< vector<int> > input_map_2d;

vector<int> extract_range;

geometry_msgs::PoseStamped local_goal;
nav_msgs::OccupancyGrid reward_map;
vector< vector<int> > reward_map_2d;


std_msgs::Float32MultiArray other_agents_state;

bool sub_global_map_flag = false;
bool sub_lcl_flag = false;
bool sub_local_goal_flag = false;
bool sub_extract_human_flag = false;
bool sub_curvature_flag = false;
bool sub_minmax_flag = false;


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



void view_2d_array(vector< vector<int> > array)
{	
	size_t array_size1 = array.size(); 
	// cout<<"array_size1 : "<<array_size1<<endl;
	size_t array_size2 = array[0].size(); 
	// cout<<"array_size2 : "<<array_size2<<endl;
	for(size_t i=0; i<array_size1; i++){
		printf("| ");
		for(size_t j=0; j<array_size2; j++){
			printf("%3d ", array[i][j]);
		}
		printf("|\n");
	}
}

void view_array(vector<int> array)
{	
	size_t array_size1 = array.size(); 
	// cout<<"array_size1 : "<<array_size1<<endl;
	printf("[ ");
	for(size_t i=0; i<array_size1; i++){
		printf("%3d ", array[i]);
	}
	printf(" ]\n");
}

void view_array_float(vector<float> array)
{	
	size_t array_size1 = array.size(); 
	// cout<<"array_size1 : "<<array_size1<<endl;
	printf("[ ");
	for(size_t i=0; i<array_size1; i++){
		printf("%.3f ", array[i]);
	}
	printf(" ]\n");
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

void init_map_2d(vector< vector<int> > &map_2d, vector<int> extract_range_)
{
	int min_x = extract_range_[0];
	int min_y = extract_range_[1];
	int max_x = extract_range_[2];
	int max_y = extract_range_[3];

	map_2d = vector< vector<int> >(max_y-min_y, vector<int>(max_x-min_x, 0));
}

vector<int> get_discreate_extract_range(vector<int> center,int width, int height, 
										vector<int> offset)
{
	// cout<<"canter : "<<endl;
	// view_array(center);
	// cout<<"offset : "<<endl;
	// view_array(offset);
	vector<int> half_extract_range = continuous2discreate(width/2, height/2, global_map, false);
	// cout<<"half_extract_range : "<<endl;
	// view_array(half_extract_range);
	int min_x = (center[0] + offset[0]) - half_extract_range[0];
	int min_y = (center[1] + offset[1]) - half_extract_range[1];
	int max_x = (center[0] + offset[0]) + half_extract_range[0];
	int max_y = (center[1] + offset[1]) + half_extract_range[1];
	vector<int> discreate_extract_range{min_x, min_y, max_x, max_y};
	// cout<<"discreate_extract_range : "<<endl;
	// view_array(discreate_extract_range);

	return discreate_extract_range;
}

void extract_grid_2d(vector< vector<int> > input, vector< vector<int> > &output, 
					   vector<int> extract_range_)
{
	init_map_2d(output, extract_range_);
	for(int y=extract_range_[1]; y<extract_range_[3]; y++){
		for(int x=extract_range_[0]; x<extract_range_[2]; x++){
			output[y-extract_range_[1]][x-extract_range_[0]] = input[y][x];
		}
	}
}


void extract_base_input_grid_map(vector< vector<int> > &base_grid_map, 
								 double offset_x, double offset_y)
{
	vector<int> global_center = continuous2discreate(-1.0*global_map.info.origin.position.x, 
													 -1.0*global_map.info.origin.position.y, 
													 global_map, false);
	vector<int> discreate_offset = continuous2discreate(offset_x, offset_y, global_map, false);

	// view_array(global_center);
	extract_range = get_discreate_extract_range(global_center, Width, Height, discreate_offset);

	extract_grid_2d(global_map_2d, base_grid_map, extract_range);
}

void set_init_grid_map(nav_msgs::OccupancyGrid &map)
{
	map.header.frame_id = "/map";
	map.header.stamp = global_map.header.stamp;
	map.info.resolution = Resolution;
	map.info.width = Width / Resolution;
	map.info.height = Height / Resolution;

	map.info.origin.position.x = -Width / 2;
	map.info.origin.position.y = -Height / 2;

	map.data.resize(map.info.width*map.info.height);
}

void set_origin_grid_map(nav_msgs::OccupancyGrid &map, nav_msgs::Odometry odom)
{
	map.info.origin.position.x = odom.pose.pose.position.x - Width / 2;
	map.info.origin.position.y = odom.pose.pose.position.y - Height / 2;
}

void set_grid_data(nav_msgs::OccupancyGrid &map, vector<int> map_1d)
{
	size_t map_size = map.data.size();
	for(size_t i=0; i<map_size; i++){
		map.data[i] = map_1d[i];
	}
}

void lclCallback(nav_msgs::Odometry msg)
{
	sub_lcl_flag = true;
	// cout<<"============ lclMapCallback ============"<<endl;

	lcl = msg;
	extract_base_input_grid_map(static_input_map_2d, 
								lcl.pose.pose.position.x, lcl.pose.pose.position.y);

	set_origin_grid_map(static_input_map, lcl);
	set_origin_grid_map(real_input_map, lcl);
	set_origin_grid_map(input_map, lcl);
	set_origin_grid_map(reward_map, lcl);

	// cout<<"========= lclCallback end ========="<<endl;
}

void globalMapCallback(nav_msgs::OccupancyGrid msg)
{
	sub_global_map_flag = true;
	// cout<<"============ globalMapCallback ============"<<endl;

	global_map = msg;
	cout<<"global_map.info.width : "<<global_map.info.width<<endl;
	cout<<"global_map.info.height : "<<global_map.info.height<<endl;
	cout<<"global_map.info.resolution : "<<global_map.info.resolution<<endl;
	cout<<"global_map.info.origin.position.x : "<<global_map.info.origin.position.x<<endl;
	cout<<"global_map.info.origin.position.y : "<<global_map.info.origin.position.y<<endl;

	global_map_2d = reshape_2dim(global_map.data, global_map.info.height, global_map.info.width);

	set_init_grid_map(static_input_map);
	set_init_grid_map(real_input_map);
	set_init_grid_map(input_map);
	set_init_grid_map(reward_map);

	// cout<<"========= globalMapCallback end ========="<<endl;
}


void cvt_pointcloud(tf::TransformListener &tflistener, 
					sensor_msgs::PointCloud &input, sensor_msgs::PointCloud &output)
{
	try{
		tflistener.waitForTransform("/map", "/velodyne2", input.header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud("/map", input.header.stamp, input, "/velodyne2", output);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s\n", ex.what());
	}

}


void pc2grid(sensor_msgs::PointCloud &input, vector< vector<int> > &map_2d)
{
	size_t input_size = input.points.size();
	for(size_t i=0; i<input_size; i++){
		vector<int> tmp_discreate = continuous2discreate(input.points[i].x, 
														 input.points[i].y, 
														 global_map, true);
		if((extract_range[0]<tmp_discreate[0] && tmp_discreate[0]<extract_range[2])
				&& (extract_range[1]<tmp_discreate[1] && tmp_discreate[1]<extract_range[3])){
			map_2d[tmp_discreate[1]-extract_range[1]][tmp_discreate[0]-extract_range[0]] = _LETHAL;
		}
	}
}

boost::mutex mutex_curvature;
sensor_msgs::PointCloud curvature_in;
void CurvatureCallback(sensor_msgs::PointCloud msg){
	boost::mutex::scoped_lock(mutex_curvature);
	curvature_in = msg;
	// cout<<"msg.header : "<<msg.header<<endl;
	//cout<<"curv_callback"<<endl;
	sub_curvature_flag = true;
}

boost::mutex mutex_minmax;
sensor_msgs::PointCloud minmax_in;
void MinMaxCallback(sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_minmax);
	sensor_msgs::PointCloud2 pc2;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, minmax_in);

	sub_minmax_flag = true;
}

void concat_grid_map(vector< vector<int> > &static_2d, vector< vector<int> > &real_2d, 
					 vector< vector<int> > &output_2d)
{
	init_map_2d(output_2d, extract_range);
	size_t rows = static_2d.size();
	size_t cols = static_2d[0].size();
	for(size_t y=0; y<rows; y++){
		for(size_t x=0; x<cols; x++){
			if(static_2d[y][x] != 0){
				output_2d[y][x] = static_2d[y][x];
			}
			if(real_2d[y][x] != 0){
				output_2d[y][x] = real_2d[y][x];
			}
		}
	}
}

void create_reward_map(geometry_msgs::PoseStamped &local_goal, 
					   vector< vector<int> > &reward_2d, 
					   vector< vector<int> > &expand_2d)
{
	vector<int> discreate_local_goal = continuous2discreate(local_goal.pose.position.x, 
															local_goal.pose.position.y, 
															global_map, true);
	// cout<<"discreate_local_goal : "<<endl;
	// view_array(discreate_local_goal);

	vector<int> discreate_lcl = continuous2discreate(lcl.pose.pose.position.x, 
													 lcl.pose.pose.position.y, 
													 global_map, true);
	// cout<<"discreate_lcl : "<<endl;
	// view_array(discreate_lcl);

	int diff_x = discreate_local_goal[0] - discreate_lcl[0];
	int diff_y = discreate_local_goal[1] - discreate_lcl[1];
	// cout<<"diff_x : "<<diff_x<<endl;
	// cout<<"diff_y : "<<diff_y<<endl;
	int x_index = discreate_local_goal[0]-extract_range[0];
	int y_index = discreate_local_goal[1]-extract_range[1];
	// cout<<"x_index : "<<x_index<<endl;
	// cout<<"y_index : "<<y_index<<endl;
	// if(diff_x > 15){
		// x_index -= 5;	
	// }
	// else if(diff_x < -15){
		// x_index += 5;
	// }

	// if(diff_y > 15){
		// y_index -= 5;
	// }
	// else if(diff_y < -15){
		// y_index += 5;
	// }
	// cout<<"x_index : "<<x_index<<endl;
	// cout<<"y_index : "<<y_index<<endl;

	init_map_2d(reward_2d, extract_range);
	int rows = reward_2d.size();
	int count = 0;
	while(count <= (int)0.5*rows){
		if(expand_2d[y_index][x_index] != _LETHAL){
			break;
		}
		else{
			// cout<<"onstacle exist!!!"<<endl;
			if(diff_x > 0){
				x_index = discreate_lcl[0] - extract_range[0] + 1;	
			}
			else{
				x_index = discreate_lcl[0] - extract_range[0] - 1;	
			}

			if(diff_y > 0){
				y_index = discreate_lcl[1] - extract_range[1] + 1;
			}
			else{
				y_index = discreate_lcl[1] - extract_range[1] - 1;
			}
		}
	}

	reward_2d[y_index][x_index] = _LETHAL;
	// cout<<"reward_2d : "<<endl;
	// view_2d_array(reward_2d);
}

void localGoalCallback(geometry_msgs::PoseStamped msg)
{
	// cout<<"============ localGoalCallback ============"<<endl;

	sub_local_goal_flag = true;
	local_goal = msg;

	// cout<<"============ localGoalCallback end ============"<<endl;
}


void extractHumanCallback(visualization_msgs::MarkerArray msg)
{
	sub_extract_human_flag = true;
	// cout<<"============ extractHumanCallback ============"<<endl;

	other_agents_state.data.clear();

	int num_human = msg.markers.size();
	cout<<"num_human : "<<num_human<<endl;
	for(int i=0; i<num_human; i++){
		vector<int> tmp_discreate = continuous2discreate(msg.markers[i].pose.position.x, 
														 msg.markers[i].pose.position.y, 
														 global_map, true);
		float yaw = tf::getYaw(msg.markers[i].pose.orientation);
		float x = tmp_discreate[0] - extract_range[0];
		float y = tmp_discreate[1] - extract_range[1];

		float max_diff_x = extract_range[2] - extract_range[0];
		float max_diff_y = extract_range[3] - extract_range[1];
		// cout<<"max_diff_x : "<<max_diff_x<<endl;
		// cout<<"max_diff_y : "<<max_diff_y<<endl;
		if(x < 0.0){
			x = 0.0;
		}
		if(y < 0.0){
			y = 0.0;
		}
		if(x >= max_diff_x){
			x = max_diff_x-1;
		}
		if(y >= max_diff_y){
			y = max_diff_y-1;
		}
		vector<float> other_agent_state{x, y, yaw};
		cout<<"other_agent_state : "<<endl;
		view_array_float(other_agent_state);
		
		for(size_t j=0; j<other_agent_state.size(); j++){
			other_agents_state.data.push_back(other_agent_state[j]);
		}
	}

	// cout<<"============ extractHumanCallback end ============"<<endl;
}


int main(int argc,char** argv)
{

	ros::init(argc, argv, "create_input_grid_map");
    ros::NodeHandle n;
	ros::Rate loop_rate(20);

	ros::Subscriber lcl_sub = n.subscribe("/lcl5", 1, lclCallback);
	ros::Subscriber global_map_sub = n.subscribe("/map", 1, globalMapCallback);
	ros::Subscriber sub_real = n.subscribe("/velodyne2/curvature",1,CurvatureCallback);
	ros::Subscriber sub_min_max = n.subscribe("/velodyne2/rm_ground2",1,MinMaxCallback);

	ros::Subscriber sub_local_goal = n.subscribe("/local_goal", 1, localGoalCallback);
	ros::Subscriber sub_extract_human = n.subscribe("/extract_human", 1, extractHumanCallback);

	ros::Publisher pub_map_static 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin/static", 1);
	ros::Publisher pub_map_static_expand 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin/static/expand", 1);
	ros::Publisher pub_map_real 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin/real", 1);
	ros::Publisher pub_map_real_expand 
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin/real/expand", 1);
	ros::Publisher pub_map
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin", 1);
	ros::Publisher pub_map_expand
		= n.advertise<nav_msgs::OccupancyGrid>("/input_grid_map/vin/expand", 1);
	ros::Publisher pub_reward_map
		= n.advertise<nav_msgs::OccupancyGrid>("/reward_map/vin", 1);

	ros::Publisher pub_other_agents_state 
		= n.advertise<std_msgs::Float32MultiArray>("/other_agents_state", 1);

	ros::Publisher pub_curvature_debug 
		= n.advertise<sensor_msgs::PointCloud>("/curvature/debug", 1);
	ros::Publisher pub_minmax_debug 
		= n.advertise<sensor_msgs::PointCloud>("/minmax/debug", 1);
	// ros::Publisher pub_map_expand 
		// = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/vin/expand", 1);
	
	tf::TransformListener tflistener;


	ExpandMap ex_map(Expand_radius, Resolution);
	ExpandMap ex_map_static(Expand_radius, Resolution);
	ExpandMap ex_map_real(Expand_radius, Resolution);


	cout<<"Here we go!!"<<endl;


	while (ros::ok()){
		if(sub_lcl_flag && sub_global_map_flag && sub_local_goal_flag && sub_extract_human_flag){
			// clock_t start=clock();
			vector<int> base_input_map_1d = reshape_1dim(static_input_map_2d);
			set_grid_data(static_input_map, base_input_map_1d);
			pub_map_static.publish(static_input_map);
			ex_map_static.expandObstacle(static_input_map);
			pub_map_static_expand.publish(ex_map_static.local_map);
			vector< vector<int> > ex_map_static_2d;
			ex_map_static_2d = reshape_2dim(ex_map_static.local_map.data, 
											ex_map_static.local_map.info.height, 
											ex_map_static.local_map.info.width);


			init_map_2d(real_input_map_2d, extract_range);
			if(sub_curvature_flag){
				sensor_msgs::PointCloud curv; //2016/1/24
				{
					boost::mutex::scoped_lock(mutex_static_);
					curv = curvature_in;
				}
				sensor_msgs::PointCloud curvature_global;
				cvt_pointcloud(tflistener, curv, curvature_global);
				pub_curvature_debug.publish(curvature_global);
				pc2grid(curvature_global, real_input_map_2d);
			}
			if(sub_minmax_flag){
				sensor_msgs::PointCloud minmax;
				{
					boost::mutex::scoped_lock(mutex_minmax);
					minmax = minmax_in;
				}
				sensor_msgs::PointCloud minmax_global;
				cvt_pointcloud(tflistener, minmax, minmax_global);
				pub_minmax_debug.publish(minmax_global);
				pc2grid(minmax_global, real_input_map_2d);
			}
			vector<int> real_input_map_1d = reshape_1dim(real_input_map_2d);
			set_grid_data(real_input_map, real_input_map_1d);
			pub_map_real.publish(real_input_map);
			ex_map_real.expandObstacle(real_input_map);
			pub_map_real_expand.publish(ex_map_real.local_map);
			vector< vector<int> > ex_map_real_2d;
			ex_map_real_2d = reshape_2dim(ex_map_real.local_map.data, 
										  ex_map_real.local_map.info.height, 
										  ex_map_real.local_map.info.width);


			concat_grid_map(static_input_map_2d, real_input_map_2d, input_map_2d);
			// concat_grid_map(static_input_map_2d, ex_map_real_2d, input_map_2d);
			vector<int> input_map_1d = reshape_1dim(input_map_2d);
			set_grid_data(input_map, input_map_1d);
			pub_map.publish(input_map);
			ex_map.expandObstacle(input_map);
			pub_map_expand.publish(ex_map.local_map);
			vector< vector<int> > ex_map_2d;
			ex_map_2d = reshape_2dim(ex_map.local_map.data, 
									 ex_map.local_map.info.height, 
									 ex_map.local_map.info.width);
			// cout<<"ex_map_2d : "<<endl;
			// view_2d_array(ex_map_2d);

			create_reward_map(local_goal, reward_map_2d, ex_map_2d);
			vector<int> reward_map_1d = reshape_1dim(reward_map_2d);
			set_grid_data(reward_map, reward_map_1d);
			pub_reward_map.publish(reward_map);

			pub_other_agents_state.publish(other_agents_state);
			
			// cout<<"duration = "<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
