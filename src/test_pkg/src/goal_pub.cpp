#include"test_pkg/goal_pub.h"

void goal_pub(const std_msgs::Int32::ConstPtr &msg)
{
    data.data = msg->data;
    goal_point = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    switch (data.data)
    {
    case 0:
        goal.header.seq = 0000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[1]+goalPoint.x2[1])/2;
        goal.pose.position.y = (goalPoint.y1[1]+goalPoint.y2[1])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 6:
        goal.header.seq = 6000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[1]+goalPoint.x2[1])/2;
        goal.pose.position.y = (goalPoint.y1[1]+goalPoint.y2[1])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 1:
        goal.header.seq = 1000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[2]+goalPoint.x2[2])/2;
        goal.pose.position.y = (goalPoint.y1[2]+goalPoint.y2[2])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 7:
        goal.header.seq = 7000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[2]+goalPoint.x2[2])/2;
        goal.pose.position.y = (goalPoint.y1[2]+goalPoint.y2[2])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 2:
        goal.header.seq = 2000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[3]+goalPoint.x2[3])/2;
        goal.pose.position.y = (goalPoint.y1[3]+goalPoint.y2[3])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3*PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 8:
        goal.header.seq = 8000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[3]+goalPoint.x2[3])/2;
        goal.pose.position.y = (goalPoint.y1[3]+goalPoint.y2[3])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3*PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 3:
        goal.header.seq = 3000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[4]+goalPoint.x2[4])/2;
        goal.pose.position.y = (goalPoint.y1[4]+goalPoint.y2[4])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3*PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 9:
        goal.header.seq = 9000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[4]+goalPoint.x2[4])/2;
        goal.pose.position.y = (goalPoint.y1[4]+goalPoint.y2[4])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3*PI/2);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 4:
        goal.header.seq = 4000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[5]+goalPoint.x2[5])/2;
        goal.pose.position.y = (goalPoint.y1[5]+goalPoint.y2[5])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 5:
        goal.header.seq = 5000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalPoint.x1[5]+goalPoint.x2[5])/2;
        goal.pose.position.y = (goalPoint.y1[5]+goalPoint.y2[5])/2;
        goal.pose.position.z = 0.0;
        qtn = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        goal.pose.orientation.x = qtn.x;
        goal.pose.orientation.y = qtn.y;
        goal.pose.orientation.z = qtn.z;
        goal.pose.orientation.w = qtn.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    case 10:
        goal.header.seq = 10000;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = odom.pose.pose.position.x;
        goal.pose.position.y = odom.pose.pose.position.y;
        goal.pose.position.z = odom.pose.pose.position.z;
        goal.pose.orientation.x = odom.pose.pose.orientation.x;
        goal.pose.orientation.y = odom.pose.pose.orientation.y;
        goal.pose.orientation.z = odom.pose.pose.orientation.z;
        goal.pose.orientation.w = odom.pose.pose.orientation.w;
        tf_listener.transformPose("odom", ros::Time(0) , goal, "map" ,odom_goal);
        goal_point.publish(odom_goal);
        break;
    default:
        break;
    }
};

void odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odomMsg)
{
	odom = *odomMsg;
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"goal_pub");
    odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1,odomCB);
    locate_sub = nh.subscribe<std_msgs::Int32>("/qingzhou_locate",1,goal_pub);
    ros::spin();
    return 0;
}
