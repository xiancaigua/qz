#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>


enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown
};


ros::Publisher qz_cmd_vel_pub;
ros::Subscriber location_sub, cmd_vel_l1_sub, cmd_vel_vision_sub;
ros::Timer timer;

geometry_msgs::Twist cmd_from_l1,cmd_from_vision,final_cmd;
std_msgs::Int32 robotlocation;

void locateCB(const std_msgs::Int32::ConstPtr& msg);
void l1_CB(const geometry_msgs::Twist::ConstPtr& msg);
void vision_CB(const geometry_msgs::Twist::ConstPtr& msg);
void pub_CB(const ros::TimerEvent&);
