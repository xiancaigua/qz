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
	// catkin_make -j8  -DCATKIN_WHITELIST_PACKAGES="dynamic_parameters"
	qingzhou_nav::L1_dynamicConfig config1;
	qingzhou_nav::L1_dynamicConfig config2;
	qingzhou_nav::L1_dynamicConfig config3;
	qingzhou_nav::L1_dynamicConfig config4;
	qingzhou_nav::L1_dynamicConfig config5;
	qingzhou_nav::L1_dynamicConfig config6;
	qingzhou_nav::L1_dynamicConfig config7;
	ParametersConfig()
	{
		// Start开始
		config1.L = 0.3;
		config1.Lrv = 1.0;
		config1.Vcmd = 0.7;
		config1.Lfw = 1.0;
		config1.lfw = 0.13;
		config1.lrv = 10.0;
		config1.controller_freq = 20.0;
		config1.angle_gain_forward = 3.8;
		config1.angle_gain_back = 0.113;
		config1.gas_gain = 1.0;
		config1.base_speed_forward = 1.0;
		config1.base_speed_back = 0.5;
		config1.base_angle = 1.0;
		config1.AllowReverse = false;
	
		config2.L = 0.3;
		config2.Lrv = 1.0;
		config2.Vcmd = 0.7;
		config2.Lfw = 0.7;
		config2.lfw = 0.13;
		config2.lrv = 10.0;
		config2.controller_freq = 20.0;
		config2.angle_gain_forward = 1.3;
		config2.angle_gain_back = 0.06;
		config2.gas_gain = 1.0;
		config2.base_speed_forward = 1.0;
		config2.base_speed_back = 0.5;
		config2.base_angle = 1.0;
		config2.AllowReverse = false;

		config3.L = 0.3;
		config3.Lrv = 1.0;
		config3.Vcmd = 0.7;
		config3.Lfw =0.9;//0.72
		config3.lfw = 0.12;
		config3.lrv = 10.0;
		config3.controller_freq = 20.0;
		config3.angle_gain_forward = 2.35;//2.4
		config3.angle_gain_back = 0.1;
		config3.gas_gain = 1.0;
		config3.base_speed_forward = 0.65;//0.55
		config3.base_speed_back = 0.6;
		config3.base_angle = 1.0;
		config3.AllowReverse = false;

		// config3.L = 0.3;
		// config3.Lrv = 1.0;
		// config3.Vcmd = 0.7;
		// config3.Lfw =0.7;//0.7
		// config3.lfw = 0.12;
		// config3.lrv = 10.0;
		// config3.controller_freq = 20.0;
		// config3.angle_gain_forward = 2.2;//2.0
		// config3.angle_gain_back = 0.1;
		// config3.gas_gain = 1.0;
		// config3.base_speed_forward = 0.65;//0.55
		// config3.base_speed_back = 0.6;
		// config3.base_angle = 1.0;
		// config3.AllowReverse = false;

		config4.L = 0.3;
		config4.Lrv = 0.6;
		config4.Vcmd = 0.7;
		config4.Lfw =1.13;//0.9
		config4.lfw = 0.13;
		config4.lrv = 10.0;
		config4.controller_freq = 20.0;
		config4.angle_gain_forward = 4.0;
		config4.angle_gain_back = 1.8;
		config4.gas_gain = 1.6;
		config4.base_speed_forward = 1.0;//0.75
		config4.base_speed_back = 0.65; 
		config4.base_angle = 1.0;
		config4.AllowReverse = false;
		// catkin_make -j8  -DCATKIN_WHITELIST_PACKAGES="dynamic_parameters"

		//这一段是去中间点
		config5.L = 0.3;
		config5.Lrv = 1.0;
		config5.Vcmd = 0.7;
		config5.Lfw = 0.7;
		config5.lfw = 0.13;
		config5.lrv = 0.0;
		config5.controller_freq = 20.0;
		config5.angle_gain_forward = 3.0;
		config5.angle_gain_back = 0.65;
		config5.gas_gain = 1.2;
		config5.base_speed_forward = 0.8;
		config5.base_speed_back = 0.35;
		config5.base_angle = 0.0;
		config5.AllowReverse = false;

		//这一段是去倒车区
		//lrv负的越多越往上
		config6.L = 0.3;
		config6.Lrv = 0.85;
		config6.Vcmd = 0.7;
		config6.Lfw = 0.4;
		config6.lfw = 0.13;
		config6.lrv = 0.08;//0.05
		config6.controller_freq = 20.0;
		config6.angle_gain_forward = 2.0;
		config6.angle_gain_back = 1.5;
		config6.gas_gain = 1.2;
		config6.base_speed_forward = 0.55;
		config6.base_speed_back = 0.65;
		config6.base_angle = 0.0;
		config6.AllowReverse = true;

		//这一段是回起始点
		config7.L = 0.3;
		config7.Lrv = 1.0;
		config7.Vcmd = 0.7;
		config7.Lfw = 1.0;
		config7.lfw = 0.13;
		config7.lrv = 0.0;
		config7.controller_freq = 20.0;
		config7.angle_gain_forward = 2.2;
		config7.angle_gain_back = 0.65;
		config7.gas_gain = 1.2;
		config7.base_speed_forward = 1.0;
		config7.base_speed_back = 0.45;
		config7.base_angle = 0.0;
		config7.AllowReverse = false;
	}
};

enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown,RoadLineToReverse,Reverse,ReverseToStart
};

class DynamicParameters
{
private:
	ros::NodeHandle nh;
	ros::Subscriber locate_sub,odom_sub;
	ros::ServiceClient map_client;
	RobotLocation robotLocation;
	ParametersConfig paramConfig;
	dynamic_reconfigure::Config costmapConfUnload;
	dynamic_reconfigure::Config costmapConfStart;
	dynamic_reconfigure::Config costmapConfLoad;
	dynamic_reconfigure::Config costmapConfTraffic;
	dynamic_reconfigure::Config costmapConfRoadLine;
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;


	std_srvs::Empty empty;
	dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig>* client;
	geometry_msgs::PoseWithCovarianceStamped odom_msg;

	void locateCB(const std_msgs::Int32& data);

public:
	DynamicParameters();
	~DynamicParameters();
	void setParameters(qingzhou_nav::L1_dynamicConfig& config);
	void initCostmapConf();
	void setCostmapConf(dynamic_reconfigure::Config& conf);
	void odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &odomMsg);

};

void dynCallBack(const qingzhou_nav::L1_dynamicConfig &data);

#endif
