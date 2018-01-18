#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>
#include <sstream>
#include <iostream>

using namespace std;


nav_msgs::OccupancyGrid global_map;
sensor_msgs::PointCloud crcl_pc;


bool sub_global_map = false;
bool sub_crcl_points = false;

void globalMapCallback(nav_msgs::OccupancyGrid msg)
{
	if(!sub_global_map)
	{
		global_map = msg;
	}

	sub_global_map = true;
	// cout<<"Subscribe global_map!!"<<endl;
}

void crclPointsCallback(sensor_msgs::PointCloud msg)
{
	sub_crcl_points = true;

	crcl_pc = msg;
	// cout<<"Subscribe crcl_pc!!"<<endl;
}

vector<int> continuous2discreate(double x, double y)
{
	double resolution = global_map.info.resolution;
	double origin_x = -1.0 * global_map.info.origin.position.x;
	double origin_y = -1.0 * global_map.info.origin.position.y;
	cout<<"origin_x : "<<origin_x<<endl;
	cout<<"origin_y : "<<origin_y<<endl;
	cout<<"resolution : "<<resolution<<endl;
	
	cout<<"x : "<<x<<endl;
	cout<<"y : "<<y<<endl;
	int p = (x + origin_x) / resolution;
	cout<<"p : " <<p<<endl;
	int q = (y + origin_y) / resolution;
	cout<<"q : " <<q<<endl;
	vector<int> output;
	output.resize(2);
	output[0] = p;
	output[1] = q;
	
	return output;
}


nav_msgs::OccupancyGrid crcl_grid_map(nav_msgs::OccupancyGrid grid_map, 
									  sensor_msgs::PointCloud crcl_points)
{
	nav_msgs::OccupancyGrid crcl_map;
	crcl_map = global_map;
	double max_x = crcl_points.points[0].x;
	double min_x = crcl_points.points[0].x;
	double max_y = crcl_points.points[0].y;
	double min_y = crcl_points.points[0].y;
	size_t crcl_points_size = crcl_points.points.size();
	for(size_t i=1; i<crcl_points_size; i++){
		double x = crcl_points.points[i].x;
		double y = crcl_points.points[i].y;

		if(x < min_x){
			min_x = x;
		}
		if(max_x < x){
			max_x = x;
		}

		if(y < min_y){
			min_y = y;
		}
		if(max_y < y){
			max_y = y;
		}
	}
	cout<<"min_x : "<<min_x<<endl;
	cout<<"max_x : "<<max_x<<endl;
	cout<<"min_y : "<<min_y<<endl;
	cout<<"max_y : "<<max_y<<endl;
	vector<int> min_point;
	min_point = continuous2discreate(min_x, min_y);
	cout<<"min_point : "<<min_point[0]<<", "<<min_point[1]<<endl;
	vector<int> max_point;
	max_point = continuous2discreate(max_x, max_y);
	cout<<"max_point : "<<max_point[0]<<", "<<max_point[1]<<endl;
	for(int x=min_point[0]; x<=max_point[0]; x++){
		for(int y=min_point[1]; y<=max_point[1]; y++){
			int num = x + y * global_map.info.width;
			crcl_map.data[num] = 0;
		}
	}

	return crcl_map;
}

void saveOccupancyGrid(const nav_msgs::OccupancyGrid& map, const string& mapname_)
{
	ROS_INFO("Map Info %d X %d map : %.3f m/pix",
			 map.info.width,
			 map.info.height,
			 map.info.resolution);
	
	string mapdatafile = mapname_ + ".pgm";
	ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
	FILE* out = fopen(mapdatafile.c_str(), "w");
	if(!out){
		ROS_ERROR("Cloud't save map file to %s", mapdatafile.c_str());
		exit(1);
	}
	
	fprintf(out, "P5\n# CREATOR: Map_generator.cpp  %.3f m/pix\n%d %d\n255\n",
			map.info.resolution, map.info.width, map.info.height);
	for(unsigned int y=0; y < map.info.height; y++){
		for(unsigned int x=0; x < map.info.width; x++){
			unsigned int i = x + (map.info.height - y -1) * map.info.width;
			if(map.data[i] == 0){
				fputc(-1.0, out);
			}else if(map.data[i] == 100){
				fputc(0.0, out);
			}else{
				fputc(100.0, out);
			}
		}
	}

	fclose(out);
	
	std::string mapmetadatafile = mapname_ + ".yaml";
	ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
	FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

	
	geometry_msgs::Quaternion orientation = map.info.origin.orientation;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	double yaw, pitch, roll;
	mat.getEulerYPR(yaw, pitch, roll);

	yaw = 0.0; // ark set

	fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
	//fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0.1\noccupied_thresh: 100\nfree_thresh: 0.0\n\n",
			mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

	fclose(yaml);

	ROS_INFO("Done\n");
	//saved_map_ = true;
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "global_obs_map_crcl");
	ros::NodeHandle n;

	ros::Subscriber global_map_sub = n.subscribe("/dilated_obs_map", 1, globalMapCallback);
	ros::Subscriber crcl_points_sub = n.subscribe("/points_4", 1, crclPointsCallback);

	ros::Publisher crcl_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/crcl_obs_map", 1);
	
	ros::Rate loop_rate(1);
	
	nav_msgs::OccupancyGrid crcl_map;
	
	int crcl_count = 0;
	int max_crcl_count = 10;
	while(ros::ok()){
		crcl_map = global_map;
		if(sub_crcl_points && sub_global_map){
			crcl_map = crcl_grid_map(global_map, crcl_pc);

			string filename = "crcl_global_grid_map_";
			int a = crcl_count % max_crcl_count;
			ostringstream ss;
			ss << a;
			filename = filename + ss.str();
			cout<<"filename : "<<filename<<endl;

			saveOccupancyGrid(crcl_map, filename);


			global_map = crcl_map;
			crcl_count++;

			sub_crcl_points = false;
		}
		crcl_map_pub.publish(crcl_map);

		loop_rate.sleep();
		ros::spinOnce();
	}



	return 0;
}
