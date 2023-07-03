/******************************************
文件名：qingzhou_locate.h

功  能：机器人当前坐标和目标点坐标
		根据当前位置和目标点位置来
		设置RobotLocation信息

作者：胡杨
******************************************/

/************************************************************************************
Read HY: 看cpp，可能会弃用Service
************************************************************************************/

#ifndef __QINGZHOU_LOCATE__
#define __QINGZHOU_LOCATE__

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown
};

// start 0.20 0.026(0 0)
// 		traffic 1.076 -	4.335 unload - 2.775 - 4.825 load 1.105 - 3.282

	// 用于划定目标点范围，可以写一个设置目标点的函数
struct GoalPoint
{
	float x1[5];
	float y1[5];
	float x2[5];
	float y2[5]; 
	GoalPoint(){
	    x1[0]= -0.5; y1[0] = -0.5; x2[0] = 0.5; y2[0] = 0.5;
		x1[1] = 1.5; y1[1] = -4.0; x2[1] = 2.5; y2[1] = -3.0;
		x1[2] = 1.5; y1[2] = -6.6; x2[2] = 2.5; y2[2] = -5.6;
		x1[3] = -2.4; y1[3] = -6.4; x2[3] = -1.4; y2[3] = -5.4;
		x1[4] = 0.6; y1[4] = -4.5; x2[4] = 1.6; y2[4] = -3.5; 
	}
};

class Locate
{
private:
	//ros句柄
	ros::NodeHandle nh;

	//机器人位置数据
	//坐标信息用于判断，可以不存储在类里
	RobotLocation robotLocation;
	RobotLocation lastLocation;
	RobotLocation goalLocation;
	GoalPoint goalPoint;
	geometry_msgs::PoseWithCovarianceStamped odom;
	geometry_msgs::PoseStamped goal;

	//话题发布订阅
	ros::Subscriber odom_sub;
	ros::Subscriber goal_sub;
	ros::Subscriber locate_sub;
	ros::Publisher location_pub;

	//回调函数（服务）

	//回调函数（话题）
	void odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odomMsg);
	void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
	void locateCB(const std_msgs::Int32& data);

public:
	Locate();
	void locationPub();
};

#endif
