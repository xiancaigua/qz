#include"ros/ros.h"
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#define PI 3.14159265358979
enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown
};
struct GoalPoint
{
	float x1[5];
	float y1[5];
	float x2[5];
	float y2[5];
	GoalPoint(){
		x1[0] = -0.5; y1[0] = -0.5; x2[0] = 0.5; y2[0] = 0.5;
		x1[1] = 2.0; y1[1] = -3.5; x2[1] = 2.8; y2[1] = -2.7;
		x1[2] = 2.0; y1[2] = -5.5; x2[2] = 2.8; y2[2] = -5.0;
		x1[3] = -2.1; y1[3] = -6.3; x2[3] = -1.6; y2[3] = -5.4;
		x1[4] = 0.5; y1[4] = -4.4; x2[4] = 1.7; y2[4] = -3.1;
	}
};
ros::NodeHandle nh;
ros::Publisher goal_point;
ros::Subscriber locate_sub;
ros::Subscriber odom_sub;
tf::TransformListener tf_listener;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped odom_goal;
std_msgs::Int32 data;
geometry_msgs::Quaternion qtn;
GoalPoint goalPoint;
geometry_msgs::PoseWithCovarianceStamped odom;