
#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace Eigen;

// minmax parameter
float minmax_low  = 0.35;
float minmax_high = 1.5; // 1.5

const float CURVATURE =  0.12; // 0.12
const float NORMAL_Z  =  0.80; //[0.80] 天井は1.0 (perfect_velodyneの法線version) // pcl:normal_estimationは天井が-1.0
//const float NORMAL_Z = 0.90;

// const float RES = 0.1;//0.1
const float RES = 0.25;//0.1
const int TRAV = 0;
const int OBS = 100;
//const int UNKNOWN = 0;
const int UNKNOWN = -1;

float max_x = 0.0;
float min_x = 0.0;
float max_y = 0.0;
float min_y = 0.0;
float min_z = 0.0;
float WIDTH = 0.0;
float HEIGHT = 0.0;

const float local_w = 20.0;
const float local_h = 20.0;

const int STEP1 = 2; // noise filter step
const int STEP2 = 2; // dilate step

geometry_msgs::PoseStamped click_pt_in;
nav_msgs::OccupancyGrid obs_map;
nav_msgs::OccupancyGrid dilated_obs_map;
bool click_callback_flag = false;


struct Cell{
	float z;
	float normal_z;
	float curv;
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

void ClickedPointCallback(const geometry_msgs::PoseStamped::Ptr& msg)
{
	click_callback_flag = true;
	click_pt_in = *msg;	
}

float MIN(float a, float b){
	if(a < b) return a;
	else return b;
}

float MAX(float a, float b){
	if(a > b) return a;
	else return b;
}

void setObsParam(const pcl::PointCloud<pcl::PointSurfel>::Ptr cloud)
{
	size_t SIZE = cloud->points.size();
	//occupancy grid map's scale
	for(size_t i=0; i<SIZE; i++){
		max_x = MAX(cloud->points[i].x, max_x);
		min_x = MIN(cloud->points[i].x, min_x);
		max_y = MAX(cloud->points[i].y, max_y);
		min_y = MIN(cloud->points[i].y, min_y);
	}

	WIDTH  = max_x - min_x;
	HEIGHT = max_y - min_y;
	//cout<<max_x<<" "<<" "<<min_x<<" "<<max_y<<" "<<min_y<<endl;
	//cout<<WIDTH<<" "<<HEIGHT<<endl;
	
	obs_map.data.resize(int(WIDTH/RES) * int(HEIGHT/RES));
	obs_map.info.width  = int(WIDTH/RES);
	obs_map.info.height = int(HEIGHT/RES);
	obs_map.info.resolution = RES;
	obs_map.info.origin.position.x = min_x;
	obs_map.info.origin.position.y = min_y;
	obs_map.info.origin.position.z = min_z;

	cout << "set param\n";
	cout << "-----------------------------------------" << "\nWIDTH: " << int(WIDTH/RES) << "\nHEIGHT: " << int(HEIGHT/RES) << "\nRESOLUTION: " <<  RES
		 << endl;
	cout << "origin position x : " << min_x << "\norigin position y : " << min_y << "\n-----------------------------------------\n";
}

void filterOcpMap(const nav_msgs::OccupancyGrid& input, nav_msgs::OccupancyGrid& output, const int& num)
{
	nav_msgs::OccupancyGrid temp, copy;
	temp = input;
	unsigned int cell_size = temp.data.size();
	unsigned int width 	   = temp.info.width;
	unsigned int height    = temp.info.height;

	for(int step=0; step<num; ++step){
		copy = temp;
		for(unsigned int i=0; i<cell_size; ++i){
			if(temp.data[i] == OBS){
				unsigned int x = i % width;
				unsigned int y = i / width;
				unsigned int num = x + y*width;
				//int trav_cnt = 0;
				int obs_cnt  = 0;
				if(x >= 1 && x < width-1 && y >= 1 && y < height-1){
					for(int row_counter = -1; row_counter <= 1; row_counter++){
						for(int col_counter = -1; col_counter <=1; col_counter++){
							if(row_counter == 0 && col_counter == 0) 
								continue;
							if( temp.data[num + width*row_counter + col_counter] == OBS) obs_cnt++;
						}
					}		
					if( obs_cnt <= 1 ) copy.data[num] = UNKNOWN; // 2
				}
			}
		}
		temp = copy; // swap
	}
	output = temp;
}

void dilateOcpMap(const nav_msgs::OccupancyGrid& input, nav_msgs::OccupancyGrid& output, const int& num)
{
	nav_msgs::OccupancyGrid temp, copy;
	temp = input;
	unsigned int cell_size = temp.data.size();
	unsigned int width     = temp.info.width;
	unsigned int height    = temp.info.height;

	for(int step=0; step<num; ++step){
		copy = temp;
		for(unsigned int i=0; i<cell_size; ++i){
			if(temp.data[i] == UNKNOWN){
				unsigned int x = i % width;
				unsigned int y = i / width;
				unsigned int num = x + y*width;
				int trav_cnt = 0;
				int obs_cnt  = 0;
				if(x >= 1 && x < width-1 && y >= 1 && y < height-1){
					for(int row_counter = -1; row_counter <= 1; row_counter++){
						for(int col_counter = -1; col_counter <=1; col_counter++){
							if(row_counter == 0 && col_counter == 0) 
								continue;
							if( temp.data[num + width*row_counter + col_counter] == TRAV) 	  trav_cnt++;
							else if( temp.data[num + width*row_counter + col_counter] == OBS) obs_cnt++;
						}
					}		
					if( trav_cnt > 0 && obs_cnt == 0) copy.data[num] = TRAV;
				}
			}
		}
		temp = copy; // swap
	}
	output = temp;
}

bool checkObsColor(unsigned char r, unsigned char g, unsigned char b)
{
	//-------------------------------------------------------------------------
	// (r,g,b) glass:(128,128,0), road(128, 64, 128), road_marker(255, 69, 0)
	// glass : road_marker = 7:3 -->  (166, 110, 0)  8:3 --> (153, 116, 0)
	// glass : road		   = 7:3 -->  (128, 109, 38) 8:3 --> (128, 115, 26)
	//-------------------------------------------------------------------------
	if( (r>=128 && r<=153) && (g>=116 && g<=128) && b==0) // glass : road_marker
		return true;
	else if(  r==128  && (g>=115 && g<=128) && b<=26)     // glass : road
		return true;
	else 
		return false;
}

//void create_obs_map(pcl::PointCloud<pcl::PointNormal> pn, nav_msgs::OccupancyGrid *obs_map){
void createObsMap(const pcl::PointCloud<pcl::PointSurfel>::Ptr cloud, nav_msgs::OccupancyGrid& obs_map){

cout<<"start!!\n";	
	nav_msgs::OccupancyGrid out;
	out = obs_map;
	
	// obs_map_init
	vector<int8_t>::iterator mit;
	for(mit=out.data.begin(); mit!=out.data.end(); mit++){
		*mit = UNKNOWN;
	}
	
	size_t SIZE = cloud->points.size();
cout << "point size ==> " << SIZE << endl;
	size_t cell_size = out.data.size();

	//std::vector<vector<float>> min_max;
	std::vector< vector<Cell> > cell_info;
	cell_info.resize(cell_size);

#pragma omp parallel for
	for(size_t i=0; i<SIZE; i++){	
		int x = int( (cloud->points[i].x - out.info.origin.position.x) / RES );
		int y = int( (cloud->points[i].y - out.info.origin.position.y) / RES );
		size_t num = x + y*out.info.width;
		Cell tmp;
		tmp.z    	 = cloud->points[i].z;
		tmp.normal_z = cloud->points[i].normal_z;
		tmp.curv 	 = cloud->points[i].curvature;
		tmp.r 	 	 = cloud->points[i].r;
		tmp.g 	 	 = cloud->points[i].g;
		tmp.b 	 	 = cloud->points[i].b;
		//if( num > 0 && num < cell_size ) min_max[num].push_back(cloud->points[i].z);
#pragma omp critical
		if( num > 0 && num < cell_size ) cell_info[num].push_back(tmp);
	}

cout << " [point] --> [cell]  FINISH!!\n";
	
	for(size_t i=0; i<cell_size; ++i){
		size_t size = cell_info[i].size();
		if( size ){
			//###float min_z    = cell_info[i][0].z; // '17.01.26
			Cell cp_cell;
			cp_cell.z 		 = cell_info[i][0].z;
			cp_cell.normal_z = cell_info[i][0].normal_z;
			int cnt_filter   = 0;
			int color_filter = 0;
			
			//-- set min_value(z) inside a cell
			//###for(size_t j=0; j<size; ++j) min_z = MIN(cell_info[i][j].z, min_z);
			for(size_t j=0; j<size; ++j){
				if( cell_info[i][j].z < cp_cell.z ) cp_cell = cell_info[i][j];
			}

			if( cp_cell.normal_z > NORMAL_Z ) continue; //### '17.01.26 最下点が天井なら処理しない
			
			//----------------------------------------------------------
			//-- check 3 obstacle (cell_info, curvature, color)   -----
			//----------------------------------------------------------
			for(size_t j=0; j<size; ++j){
			//###'17.01.26	if( cell_info[i][j].z - min_z > minmax_low && cell_info[i][j].z - min_z < minmax_high ) // min_max
				if( (cell_info[i][j].z - cp_cell.z > minmax_low) && (cell_info[i][j].z - cp_cell.z < minmax_high) ) // min_max
					cnt_filter++;
				//### '17.01.26 else if( cell_info[i][j].z - min_z <= minmax_low ){
				// else if( cell_info[i][j].z - cp_cell.z <= minmax_low ){
					// if( cell_info[i][j].curv > CURVATURE ) cnt_filter++; // curvature
					// else{
						// if( checkObsColor( cell_info[i][j].r, cell_info[i][j].g, cell_info[i][j].b )) color_filter++; // color 
					// }
				// }
			}
			if(cnt_filter >=1 || color_filter >=1) out.data[i] = OBS;
			else out.data[i] = TRAV;
		}
	}
	obs_map = out;
cout<<"set ocp\n";
	
	// ノイズ除去
	nav_msgs::OccupancyGrid filtered_out;
	filterOcpMap(out, filtered_out, STEP1);	
	// 膨張処理
	nav_msgs::OccupancyGrid dilated_out;
	dilateOcpMap(filtered_out, dilated_out, STEP2);
	
	dilated_obs_map = dilated_out;
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
				fputc(0.0, out);
			}else if(map.data[i] == 100){
				fputc(100, out);
			}else{
				fputc(-1.0, out);
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
	ros::init(argc, argv, "SIMPLE_OBSMAP_CREATER");
  	ros::NodeHandle n;
	ros::Rate roop(1);

	cout<<"Here we go!!!"<<endl;
	
	string str, output_ocp;	
	cout << "file name -> "; cin >> str;
	str += ".pcd";
	//cout << "output occupancy file == > "; cin >> output_ocp;
	//output_ocp += ".yaml";


	ros::Publisher pub_pc_all_1  = n.advertise<sensor_msgs::PointCloud2>("/org_pcd", 1);
	ros::Publisher pub_pc_all_2  = n.advertise<sensor_msgs::PointCloud2>("/filterd_pcd", 1);
	ros::Publisher pub_obs_map_1 = n.advertise<nav_msgs::OccupancyGrid>("/obs_map", 1);
	ros::Publisher pub_obs_map_2 = n.advertise<nav_msgs::OccupancyGrid>("/dilated_obs_map", 1);


	pcl::PointCloud<pcl::PointSurfel>::Ptr pcd_pnt (new pcl::PointCloud<pcl::PointSurfel>);
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcd_pnt (new pcl::PointCloud<pcl::PointXYZINormal>);
	
	sensor_msgs::PointCloud2 ros_pc, ros_pc_filtered;
	//nav_msgs::OccupancyGrid obs_map;


	if(pcl::io::loadPCDFile<pcl::PointSurfel>(str, *pcd_pnt) == -1){
		cout << "load error!!\n";
		exit(1);
	}
	cout << "loaded..\n";
	//-------------------------
	//--- Outliers removal ---- ==> 事前処理に変更(outlier_filter.cpp)
	//-------------------------
	pcl::PointCloud<pcl::PointSurfel>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointSurfel>);
	
	pcl::StatisticalOutlierRemoval<pcl::PointSurfel> sor;
	sor.setInputCloud (pcd_pnt);
	sor.setMeanK (50); 			  // default(50)
	sor.setStddevMulThresh (1.5); // default(1.0) // 0.5
	sor.filter (*cloud_filtered);
	
	cout << "filtered size ==> " << cloud_filtered->points.size() << endl;
	pcl::io::savePCDFileBinary("diff_filtered.pcd", *cloud_filtered);

	//--- set obstacle_map parameter
	//setObsParam( pcd_pnt /*cloud_filtered*/);
	setObsParam( cloud_filtered );
	//createObsMap( pcd_pnt /*cloud_filtered*/, obs_map);
	createObsMap( cloud_filtered, obs_map);
	//saveOccupancyGrid(obs_map, output_ocp);
	
	pcl::toROSMsg(*pcd_pnt, ros_pc);
	//pcl::toROSMsg(*cloud_filtered, ros_pc_filtered);
	ros_pc.header.frame_id 			= "/map";
	//ros_pc_filtered.header.frame_id = "/map";
	obs_map.header.frame_id 		= "/map";
	dilated_obs_map.header.frame_id = "/map";


	while(ros::ok()){
	 	cout << "----- Obstacle Map publish!! -----\n";	
		//to ros msgs
		ros_pc.header.stamp = ros::Time::now();
		pub_pc_all_1.publish(ros_pc);
		//ros_pc_filtered.header.stamp = ros::Time::now();
		//pub_pc_all_2.publish(ros_pc_filtered);

		obs_map.header.stamp = ros::Time::now();
		pub_obs_map_1.publish(obs_map);

		dilated_obs_map.header.stamp = ros::Time::now();
		pub_obs_map_2.publish(dilated_obs_map);
		//process is completed
		ros::spinOnce();
		roop.sleep();
	}
	
	return (0);
}
