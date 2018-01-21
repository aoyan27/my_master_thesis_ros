/* NDT_Dgauss_ekf.cpp
 * 2016.10.05
 * 
 * author : Takashi Ienaga
 * 
 * Distribution : rwrc16のD-GaussとNDTの拡張カルマンフィルタ(EKF)
 * 				  入力：Gyro-Odometry
 * 				  観測：D-Gauss
 * 					  　NDT
 */

/* 備考
 * 2016.10.05 : 各観測(マッチング)は推定値のみ．スコアは無いので，分散は固定値でパラメータとしてlaunchで設定してね
 */ 

//Library
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <sys/time.h>
#include <ekf/EKF.h>

//msgs
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace Eigen;
EKF ekf;

bool imu_flag = false;
bool odom_flag = false;
bool init_flag = true;

nav_msgs::Odometry gyro_odom;
nav_msgs::Odometry ekf_odom;

MatrixXf x(3,1);		// 状態 (x,y,θ)
MatrixXf Sigma(3,3);
MatrixXf u(2,1);		// 制御 (v, w)
MatrixXf obs_gauss(3,1);// D-gauss観測 (x,y,θ)
MatrixXf obs_ndt(3,1);	// NDT観測 (x,y,θ)
double init_x[3];		// 初期状態 (x,y,θ)
double init_sig[3];		// 初期分散 (sig_x, sig_y, sig_yaw)
double s_ndt[3]; 		// NDT観測値の分散 (sig_x, sig_y, sig_yaw)
double ndt_sig[2];
double s_input[4]; 		// 制御の誤差パラメータ (要素数[0],[2]は並進速度，[1],[3]は回頭速度のパラメータ)
float pitch;			// ピッチ角

int n = 0;
float last_angle;

MatrixXf predict(MatrixXf x, MatrixXf u, float dt, double *s_input, float pitch){
	/* u   : (v, w)の転置行列　v:並進速度, w:角速度
	 * x   : (x, y, θ)の転置行列
	 * dt	   : 前ステップからの経過時間
	 * s_input : 動作モデルのノイズパラメータ
	 */
//printf("\r prediction!!\n");
	MatrixXf Mu = MatrixXf::Zero(3,1);
	MatrixXf P = MatrixXf::Zero(3,3);
	MatrixXf Gt = MatrixXf::Zero(3,3);
	MatrixXf Vt = MatrixXf::Zero(3,2);
	MatrixXf Mt = MatrixXf::Zero(2,2);
	
/*	Mu << x.coeffRef(0.0),
		  x.coeffRef(1,0),
		  x.coeffRef(2,0);*/
	Mu = x;
	P = Sigma;
	
	Gt = ekf.jacobG(x, u, dt, pitch);
//	Gt = ekf.jacobF(x, u, dt);
	Vt = ekf.jacobV(x, u, dt, pitch);
	Mt = ekf.jacobM(u, s_input);
	Mu = ekf.move(x, u, dt, pitch);
	P = Gt*Sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
//cout << "Sigma= " << endl << Sigma << endl;
	Sigma = P;
//cout << "Sigma= " << endl << Sigma << endl;	
	
	return Mu;
}

MatrixXf NDTUpdate(MatrixXf x){
	/* x	: 状態(x, y, yaw)の転置行列
	 * u	: 制御(v, w)の転置行列
	 * s_ndt: 観測ノイズ
	 * sigma: 推定誤差
	 */
// printf(" NDT measurement update!!\n");
	MatrixXf Mu= MatrixXf::Zero(3,1);
	MatrixXf P = MatrixXf::Zero(3,3);
	MatrixXf Q= MatrixXf::Zero(3,3);
	MatrixXf H= MatrixXf::Zero(3,3);
	MatrixXf y= MatrixXf::Zero(3,1);
	MatrixXf S= MatrixXf::Zero(3,3);
	MatrixXf K= MatrixXf::Zero(3,3);
	MatrixXf I = MatrixXf::Identity(3,3);
	
	Mu = x; 
	P = Sigma;
//cout << "Mu = " << endl << Mu << endl;
//cout << "P = " << endl << P << endl;
	Q.coeffRef(0,0) = (float)s_ndt[0];
	Q.coeffRef(1,1) = (float)s_ndt[1];
	Q.coeffRef(2,2) = (float)s_ndt[2];
// cout << "Q_ndt = " << endl << Q << endl;
	H = ekf.jacobH(x);
	y = obs_ndt - ekf.h(x);							//predictによる現在地とmeasurementによる推定値の差
	S = H * Sigma * H.transpose() + Q;
	K = Sigma * H.transpose() * S.inverse();
//cout << "S = " << endl << S << endl;
//cout << "K = " << endl << K << endl;
	Mu = Mu + K*y;
	P = (I - K*H)*Sigma;
//cout << "Mu = " << endl << Mu << endl;
//cout << "P = " << endl << P << endl;
	Sigma = P;
//cout << "NDT Sigma= " << endl << Sigma << endl;
	
	return Mu;
}

float expand_angle(float now_angle){
	float angle_ = now_angle;
	if((now_angle * last_angle) < 0.0){
		if((now_angle - last_angle) < -M_PI*1.8){
			n++;
		}
		else if((now_angle - last_angle) > M_PI*1.8){
			n--;
		}
	}
	cout<<"now_angle : "<<now_angle<<endl;
	cout<<"last_angle : "<<last_angle<<endl;
	cout<<"n : "<<n<<endl;
	angle_ += 2.0 * n * M_PI;
	cout<<"angle_ : "<<angle_<<endl;
	last_angle = now_angle;
	return angle_;
}

void ndtCallback(nav_msgs::Odometry msg){
	ekf_odom.header.stamp = msg.header.stamp;
	obs_ndt.coeffRef(0,0) = msg.pose.pose.position.x;
	obs_ndt.coeffRef(1,0) = msg.pose.pose.position.y;
	obs_ndt.coeffRef(2,0) = expand_angle((float)(tf::getYaw(msg.pose.pose.orientation)));
	x= NDTUpdate(x);
}

void imuCallback(sensor_msgs::Imu msg){
	u.coeffRef(1,0) = msg.angular_velocity.z; //[rad/s]
	ekf_odom.twist.twist.angular.z = u.coeffRef(1,0);

	tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
	tf::Matrix3x3 m(q);
	double roll_, pitch_, yaw_;
	m.getRPY(roll_, pitch_, yaw_);
	pitch = pitch_;
	imu_flag = true;
}

void odomCallback(nav_msgs::Odometry msg){
	u.coeffRef(0,0) = msg.twist.twist.linear.x;
	ekf_odom.twist.twist.linear.x = u.coeffRef(0,0);
	odom_flag = true;
}


void poseInit(nav_msgs::Odometry &msg){
	msg.header.frame_id = "/map";
	// msg.child_frame_id = "/ekf";
	msg.child_frame_id = "/matching_base_link";
	msg.pose.pose.position.x = init_x[0];
	msg.pose.pose.position.y = init_x[1];
	msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_x[2] / 180.0 * M_PI);	
	

	x << init_x[0], init_x[1], init_x[2] / 180.0 * M_PI;
	cout<<"x : "<<endl<<x<<endl;
	Sigma << init_sig[0], 0, 0,
			 0, init_sig[1], 0,
			 0, 0, init_sig[2];
	u = MatrixXf::Zero(2,1);
	obs_ndt = MatrixXf::Zero(3,1);
}

void printParam(void){
	printf("Dgauss_ekf.cpp Parameters:\n");
	printf("Initial pose \n");
	printf("	init_x		: %lf\n", init_x[0]);
	printf("	init_y		: %lf\n", init_x[1]);
	printf("	init_yaw	: %lf\n", init_x[2]);
	printf("	init_sig_x	: %f\n", init_sig[0]);
	printf("	init_sig_y	: %f\n", init_sig[1]);
	printf("	init_sig_yaw	: %f\n", init_sig[2]);
	printf("Prediction \n");
	for(unsigned int i=0; i<sizeof(s_input)/sizeof(s_input[0]); i++){
		printf("	a%d		: %lf\n", i+1, s_input[i]);
	}
	printf("NDT Measurement \n");
	printf("	Sig_X		: %lf\n", s_ndt[0]);	ndt_sig[0] = s_ndt[0];
	printf("	Sig_Y		: %lf\n", s_ndt[1]);	ndt_sig[1] = s_ndt[1];
	printf("	Sig_Yaw		: %lf\n", s_ndt[2]);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "NDT_ekf");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	
	//Subscribe
	// ros::Subscriber ndt_sub   = n.subscribe("/lcl3", 1, ndtCallback);
	// ros::Subscriber ndt_sub   = n.subscribe("/lcl_ndt", 1, ndtCallback);
	ros::Subscriber ndt_sub   = n.subscribe("/vis_lcl_ndt", 1, ndtCallback);

	ros::Subscriber imu_sub   = n.subscribe("/imu/data", 1, imuCallback);
	ros::Subscriber odom_sub  = n.subscribe("/odom", 1, odomCallback);

	//Publish
	ros::Publisher ekf_pub = n.advertise<nav_msgs::Odometry>("/ekf_ndt", 1);

	float dt;
	struct timeval last_time, now_time;
	
	//パラメータ
	pnh.param<double>("init_x", init_x[0], 0.0);
	pnh.param<double>("init_y", init_x[1], 0.0);
	pnh.param<double>("init_yaw", init_x[2], 0.0);
	pnh.param<double>("init_sig_x", init_sig[0], 0.0);
	pnh.param<double>("init_sig_y", init_sig[1], 0.0);
	pnh.param<double>("init_sig_yaw", init_sig[2], 0.0);
	pnh.param<double>("Pred_a1", s_input[0], 0.0);
	pnh.param<double>("Pred_a2", s_input[1], 0.0);
	pnh.param<double>("Pred_a3", s_input[2], 0.0);
	pnh.param<double>("Pred_a4", s_input[3], 0.0);
	pnh.param<double>("NDT_sig_X", s_ndt[0], 0.0);
	pnh.param<double>("NDT_sig_Y", s_ndt[1], 0.0);
	pnh.param<double>("NDT_sig_Yaw", s_ndt[2], 0.0);
	
	printParam();
	
	//初期化
	poseInit(gyro_odom);
	poseInit(ekf_odom);
	last_angle = init_x[2] / 180.0 * M_PI;
	
	//時刻取得
	gettimeofday(&last_time, NULL);
	
	ros::Rate loop(200);
	while(ros::ok()){
		if(imu_flag && odom_flag){
			if(init_flag){
				//時刻取得
				gettimeofday(&now_time, NULL);
				dt = 0.02;
				init_flag = false;
			}else{
				//時刻取得
				gettimeofday(&now_time, NULL);
				dt = (now_time.tv_sec - last_time.tv_sec) + (now_time.tv_usec - last_time.tv_usec)*1e-6;
			}
//			printf("dt = %f\n", dt);
			
			// cout<<"x : "<<endl<<x<<endl;
			// cout<<"u : "<<endl<<u<<endl;

			x = predict(x, u, dt, s_input, pitch);
			last_time = now_time;
			imu_flag = odom_flag = false;
		}
		ekf_odom.pose.pose.position.x = x.coeffRef(0,0);
		ekf_odom.pose.pose.position.y = x.coeffRef(1,0);
		ekf_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(x.coeffRef(2,0));
		// cout << "x : " <<endl <<x << endl;
		// cout << "yaw : " << x.coeffRef(2,0) << endl;
//		vis_pub.publish(ekf_odom);
//		float yaw = (float)(tf::getYaw(ekf_odom.pose.pose.orientation));
//		ekf_odom.pose.pose.orientation.z = yaw;
//		ekf_odom.pose.pose.orientation.w = 0.0;
		// cout << "x : " << endl << "  " << x.coeffRef(0,0) << ",  " << x.coeffRef(1,0) << ",  " << x.coeffRef(2,0) << endl;
		ekf_pub.publish(ekf_odom);
		
		loop.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
