/******************************************
文件名：dynamic_parameters.h

功  能：进行动态调参

作者：胡杨
******************************************/

#ifndef __DYNAMIC_PARAMETERS__
#define __DYNAMIC_PARAMETERS__

#include <ros/ros.h>
#include <stdio.h>
#include <boost/function.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "qingzhou_locate/RobotLocation.h"
#include "dynamic_reconfigure/config_tools.h"
#include "dynamic_reconfigure/client.h"
#include "qingzhou_nav/L1_dynamicConfig.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <unistd.h> 

struct ParametersConfig
{
	// catkin_make -DCATKIN_WHITELIST_PACKAGES="dynamic_parameters"
	double limit = -7.75;//-7.62
	qingzhou_nav::L1_dynamicConfig config1;
	qingzhou_nav::L1_dynamicConfig config2;
	qingzhou_nav::L1_dynamicConfig config3;
	qingzhou_nav::L1_dynamicConfig config4;
	qingzhou_nav::L1_dynamicConfig config5;
	ParametersConfig(){
		// Start开始
		config1.y_limit = limit;
		config1.L = 0.3;
		config1.Lrv = 1.0;
		config1.Vcmd = 0.7;
		config1.Lfw = 0.55;//早上跑五圈0.55
		config1.lfw = 0.13;
		config1.lrv = 10.0;
		config1.controller_freq = 20.0;
		config1.angle_gain_forward = 1.8;//早上跑五圈的代码1.8
		config1.angle_gain_back = 0.113;
		config1.gas_gain = 1.0;
		config1.base_speed_forward = 0.55;//五0.7
		config1.base_speed_back = 0.5;
		config1.base_angle = 0.5;
		config1.AllowReverse = false;

		config2.y_limit = limit;	
		config2.L = 0.3;
		config2.Lrv = 1.0;
		config2.Vcmd = 0.7;
		config2.Lfw = 0.45;  // 0.55
		config2.lfw = 0.14;
		config2.lrv = 10.0;
		config2.controller_freq = 20.0;
		config2.angle_gain_forward = 1.35; // 0.015
		config2.angle_gain_back = 0.06; // 7月3日0.1   原来0.08
		config2.gas_gain = 1.0;
		config2.base_speed_forward = 0.52;
		config2.base_speed_back = 0.5;
		config2.base_angle = 0.0;
		config2.AllowReverse = false;

		config3.y_limit = limit;
		config3.L = 0.3; //0.27
		config3.Lrv = 1.0;
		config3.Vcmd = 0.7;
		config3.Lfw =0.33;  //0.55 0.4
		config3.lfw = 0.1;
		config3.lrv = 10.0;
		config3.controller_freq = 20.0;
		config3.angle_gain_forward = 1.3; //1.8 1.35
		config3.angle_gain_back = 0.1; 
		config3.gas_gain = 1.0;
		config3.base_speed_forward = 0.55;
		config3.base_speed_back = 0.6;
		config3.base_angle = 0.0;
		config3.AllowReverse = false;

		config4.y_limit = limit;
		config4.L = 0.25;//越大角度越大 0.23
		config4.Lrv = 0.6;
		config4.Vcmd = 0.7;
		config4.Lfw = 0.7; //0.7
		config4.lfw = 0.13;//越大角度越小
		config4.lrv = 10.0;
		config4.controller_freq = 20.0;
		config4.angle_gain_forward = 3.6; // 角度增益3.6
		config4.angle_gain_back = 1.8; // 角度增益
		config4.gas_gain = 1.6;
		config4.base_speed_forward = 0.55;//速度
		config4.base_speed_back = 0.55; // 速度
		config4.base_angle = 0.0;
		config4.AllowReverse = false;

		config5.y_limit = limit;
		config5.L = 0.3;
		config5.Lrv = 1.0;
		config5.Vcmd = 0.7;
		config5.Lfw = 0.5;
		config5.lfw = 0.13;
		config5.lrv = 10.0;
		config5.controller_freq = 20.0;
		config5.angle_gain_forward = 1.4;//1.4
		config5.angle_gain_back = 0.75;
		config5.gas_gain = 1.2;
		config5.base_speed_forward = 0.55;
		config5.base_speed_back = 0.6;
		config5.base_angle = 0.0;
		config5.AllowReverse = false;
	}
};

enum RobotLocation : int
{
	Start,
	Load,
	TrafficLight,
	Unload,
	RoadLine,
	RoadLineToStart,
	StartToLoad,
	LoadToTrafficLight,
	TrafficLightToUnload,
	UnloadToRoadLine,
	Unknown
};

class DynamicParameters
{
private:
	ros::NodeHandle nh;
	ros::Subscriber locate_sub;
	ros::ServiceClient map_client;
	RobotLocation robotLocation;
	ParametersConfig paramConfig;
	dynamic_reconfigure::Config costmapConfUnload;
	dynamic_reconfigure::Config costmapConfStart;
	dynamic_reconfigure::Config costmapConfTraffic;
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;


	std_srvs::Empty empty;
	dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig>* client;

	void locateCB(const std_msgs::Int32& data);

public:
	DynamicParameters();
	~DynamicParameters();
	void setParameters(qingzhou_nav::L1_dynamicConfig& config);
	void initCostmapConf();
	void setCostmapConf(dynamic_reconfigure::Config& conf);

	// 屎山代码的ODOM
	ros::Subscriber odom_sub;
	void odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &odomMsg);

};

// 屎山代码的ODOM
geometry_msgs::PoseWithCovarianceStamped odom_msg;


void dynCallBack(const qingzhou_nav::L1_dynamicConfig &data);

#endif
