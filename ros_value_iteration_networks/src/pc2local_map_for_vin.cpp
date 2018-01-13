#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>


#define Width 10
#define Length 10
#define Resolution 0.25

#define INV_Resolution 1.0/Resolution
#define HALF_WIDTH Width/2
#define HALF_LENGTH Length/2

#define MAP_WIDTH Width/Resolution
#define MAP_LENGTH Length/Resolution
#define MAP_SIZE MAP_WIDTH*MAP_LENGTH

#define _FREE 0
#define _UNEXPLORE 50
#define _LETHAL 100

#define CELL_FILTER 0

using namespace std;
using namespace Eigen;

boost::mutex mutex_;

bool flag = false;
bool flag1 = false;
bool flag2 = false;
bool flag3 = false;

sensor_msgs::PointCloud pc_in;
sensor_msgs::PointCloud pc2_in;

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
	void getGridCells(nav_msgs::GridCells& cells);
	
	nav_msgs::OccupancyGrid local_map;
};

// midpoint circle algorithm
ExpandMap::ExpandMap(float _radius, float resolution){
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
	//cout<<"hello"<<endl;
	
	/* 20151103 */
	/*// fill the grids in the circle
	// bresenham's line algorithm
	int n=expanded_circle.size();
	list<MapIndex>::iterator itr=expanded_circle.begin();
	//for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
	for(int i=0; i<n; i+=2){
		int x1=itr->i;
		int y1=itr->j;
		itr++;
		int x2=itr->i;
		int y2=itr->j;
		itr++;
		bool steep= abs(y2-y1)>abs(x2-x1);
	
		if(steep){
			swap(x1,y1);
			swap(x2,y2);
		}
		if(x1>x2){
			swap(x1,x2);
			swap(y1,y2);		
		}
	
		int deltax=x2-x1;
		int deltay=abs(y2-y1);	
		float error=0;
		float deltaerr;
		if(deltax==0){
			deltaerr=10000;
		}else{
			deltaerr=deltay/deltax;	
		}
		int ystep;
		int yt=y1;
		if(y1<y2){
			ystep=1;
		}else{
			ystep=-1;
		}
		for(int xt=x1; xt<=x2; xt++){
			//cout<<xt<<":"<<x1<<","<<x2<<endl;
			if(steep){
				expanded_circle.push_back(MapIndex(yt,xt));
			}else{
				expanded_circle.push_back(MapIndex(xt,yt));
			}
		
			error+=deltaerr;
		
			if(error>=0.5){
				yt+=ystep;
				error-=1;
			}	
		}	
	}
	*/


	// delete several overlap grids
	expanded_circle.sort();
	expanded_circle.unique();
	
	//list<MapIndex>::iterator itr;
	/*
	for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
		cout<<itr->i<<"\t"<<itr->j<<endl;
	}*/
}

void ExpandMap::expandObstacle(const nav_msgs::OccupancyGrid& map_in){
	local_map=map_in;
	
	vector<int8_t>::iterator itr;
	for(itr=local_map.data.begin(); itr!=local_map.data.end(); itr++){
		*itr=FREE;
	}
	
	
	for(int xi=0; xi<(int)map_in.info.height; xi++){
		for(int yi=0; yi<(int)map_in.info.width; yi++){
			// if the cell is LETHAL
			if(map_in.data[xi+map_in.info.width*yi]!=FREE){
				// expand the LETHAL cells with respect to the circle radius
				list<MapIndex>::iterator litr;
				for(litr=expanded_circle.begin(); litr!=expanded_circle.end(); litr++){
					int x=xi+litr->i, y=yi+litr->j;
					if(x>=0 && x<(int)local_map.info.height && 	y>=0 && y<(int)local_map.info.width
						&& map_in.data[xi+map_in.info.width*yi]>local_map.data[x+map_in.info.width*y]){
						local_map.data[x+map_in.info.width*y]=map_in.data[xi+map_in.info.width*yi];
					}
				}
			}
		}
	}
}

//change(2011/09/30)
void ExpandMap::getGridCells(nav_msgs::GridCells& cells){
	cells.cells.clear();
	
	cells.header.frame_id=local_map.header.frame_id;
	cells.header.stamp=local_map.header.stamp;
	cells.cell_width=local_map.info.resolution;
	cells.cell_height=local_map.info.resolution;
	
	float _map_angle = tf::getYaw(local_map.info.origin.orientation);
	float map_angle = _map_angle - M_PI;
	
	
	for(int xi=0; xi<(int)local_map.info.height; xi++){
		for(int yi=0; yi<(int)local_map.info.width; yi++){
			if(local_map.data[xi+local_map.info.width*yi]!=FREE){
				float x_conv=cos(map_angle)*xi-sin(map_angle)*yi;
				float y_conv=sin(map_angle)*xi+cos(map_angle)*yi;
				
				float x=local_map.info.origin.position.x-x_conv*local_map.info.resolution;
				float y=local_map.info.origin.position.y-y_conv*local_map.info.resolution;//*/

				geometry_msgs::Point pt;
				pt.x=x;	pt.y=y;	pt.z=0;
				cells.cells.push_back(pt);
			}
		}
	}
}

void PC2GridMap(sensor_msgs::PointCloud& pc, nav_msgs::OccupancyGrid& map)
{
	int pt_size=pc.points.size();
	double x,y;
	int p,q,q_;
	for (int i=0; i<pt_size; ++i){
		x = pc.points[i].x;
		y = pc.points[i].y;

		p = (x+HALF_WIDTH)*INV_Resolution;
		q = (y+HALF_WIDTH)*INV_Resolution;
		q_ = q*MAP_WIDTH;

		if(fabs(x)<HALF_WIDTH && fabs(y)<HALF_WIDTH){
			map.data[ p + q_ ] = 100;
		}
	}
}

void PC2GridMap_minmax(sensor_msgs::PointCloud& pc, nav_msgs::OccupancyGrid& map)
{
	int pt_size=pc.points.size();
	double x,y;
	int p,q,q_;
	vector<int> count((((int)Width * INV_Resolution) * ((int)Length * INV_Resolution)), 0);
	for (int i=0; i<pt_size; ++i){
		x = pc.points[i].x;
		y = pc.points[i].y;

		p = (x+HALF_WIDTH)*INV_Resolution;
		// cout<<"p : " <<p<<endl;
		q = (y+HALF_WIDTH)*INV_Resolution;
		// cout<<"q : " <<q<<endl;
		q_ = q*MAP_WIDTH;

		if(fabs(x)<HALF_WIDTH && fabs(y)<HALF_WIDTH){
			int num = p + q_;
			count[num] += 1;
			// map.data[ p + q_ ] = 100;
		}
	}

	for(unsigned int i=0; i < count.size(); i++){
		if(count[i] > CELL_FILTER){
			map.data[i] = 100;
		}
	}
}

boost::mutex mutex_curvature;
sensor_msgs::PointCloud curvature_in;
void CurvatureCallback(sensor_msgs::PointCloud input){
	boost::mutex::scoped_lock(mutex_curvature);
	curvature_in = input;
	//cout<<"curv_callback"<<endl;
}

boost::mutex mutex_minmax;
sensor_msgs::PointCloud minmax_in;
void MinMaxCallback(sensor_msgs::PointCloud2 input){
	boost::mutex::scoped_lock(mutex_minmax);
	sensor_msgs::PointCloud2 pc2;
	pc2 = input;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2,minmax_in);
}


int main(int argc,char** argv)
{

	ros::init(argc, argv, "pc2local_map_for_vin");
    ros::NodeHandle n;
	ros::Rate roop(20);

	ros::Subscriber sub_real = n.subscribe("/curvature",1,CurvatureCallback);
	ros::Subscriber sub_min_max = n.subscribe("/rm_ground2",1,MinMaxCallback);
	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/vin", 1);
	ros::Publisher pub_map_expand \
		= n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/vin/expand", 1);
		
	nav_msgs::OccupancyGrid map;
	map.data.resize(MAP_SIZE);
	map.info.width = MAP_WIDTH;
	map.info.height = MAP_LENGTH;
	map.info.resolution = Resolution;
	map.info.origin.position.x = -Length/2.0;
	map.info.origin.position.y = -Width/2.0;
	
	nav_msgs::GridCells cells;
	cells.cell_width = Resolution;
	cells.cell_height = Resolution;
	map.header.frame_id = cells.header.frame_id = "/velodyne";
	
	MatrixXd init;
	MatrixXd init2;
	//tsukuchalle
	// ExpandMap ex_map(0.5, 0.1);
	// ExpandMap ex_map(0.6, 0.1);
	ExpandMap ex_map(0.15, 0.1);
	// ExpandMap ex_map(0.3, 0.1);
	
	while (ros::ok()){
		//clock_t start=clock();
		sensor_msgs::PointCloud curv; //2016/1/24
		{
			boost::mutex::scoped_lock(mutex_static_);
			curv = curvature_in;
		}
		sensor_msgs::PointCloud minmax;
		{
			boost::mutex::scoped_lock(mutex_minmax);
			minmax = minmax_in;
		}
		
		//initialize//
		for (size_t i=0; i<MAP_SIZE; ++i)
			map.data[i]=_FREE;
		
		PC2GridMap(curv, map);//2016/1/24
		PC2GridMap_minmax(minmax, map);
		pub_map.publish(map);
		ex_map.expandObstacle(map);
		pub_map_expand.publish(ex_map.local_map);
		//cout<<"t:"<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;
		ros::spinOnce();
		roop.sleep();
	}
	return 0;
}
