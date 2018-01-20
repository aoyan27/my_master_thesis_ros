#ifndef __LOCALIZATION_H__
#include <time_util/stopwatch.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

class Localization{
public:
	double x,y,z,qw,qx,qy,qz,pitch,yaw,v,w;
	double time;

	Stopwatch sw;
	
	Localization();
	~Localization();
	void showState();
	void altering();
	void altering2();
	void altering3();
	void start();
};

#include "impl/localization_imu.hpp"
#endif
