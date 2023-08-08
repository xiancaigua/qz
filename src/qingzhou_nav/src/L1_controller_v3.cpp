#include <iostream>
#include "ros/ros.h"

#include "dynamic_reconfigure/server.h"
#include "qingzhou_nav/L1_dynamicConfig.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

#include <unistd.h>

#define PI 3.14159265358978

/********************/
/* CLASS DEFINITION */
/********************/

struct GoalPoint
{
    float x1[6];
    float y1[6];
    float x2[6];
    float y2[6];
    GoalPoint()
    {
        x1[0] = -0.5;y1[0] = -0.5;x2[0] = 0.5;y2[0] = 0.5;
        x1[1] = 1.7;y1[1] = -3.9;x2[1] = 2.7;y2[1] = -2.9;
        x1[2] = 1.7;y1[2] = -6.3;x2[2] = 2.7;y2[2] = -5.3;
        x1[3] = -2.7;y1[3] = -6.5;x2[3] = -1.7;y2[3] = -5.5;
        x1[4] = 0.5;y1[4] = -4.3;x2[4] = 1.5;y2[4] = -3.0;
        x1[5] = -2.9;y1[5] = -1.6;x2[5] = -1.9;y2[5] = -0.1;
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
    Unknown,
    RoadLineToReverse,
    Reverse,
    ReverseToStart
};

class L1Controller
{
public:
    L1Controller();
    ~L1Controller();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose);
    bool isWayPtAwayFromLfw_LrvDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos, double Lfw_Lrv);
    double getYawFromPose(const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPos);
    double getCar2GoalDist();
    double getL1Distance(const double &_Vcmd);
    double getSteeringAngle(double eta, double Lfw_Lrv, double lfw_lrv);
    double getGasInput(const float &current_v);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);

private:
    ros::NodeHandle n_;

    RobotLocation goalLocation;
    GoalPoint goalPoint;

    geometry_msgs::PoseStamped goal,goal_;

    ros::Subscriber odom_sub,path_sub, goal_sub, hypath_sub, locate_sub;
    ros::Publisher pub_, marker_pub, planner_pub, locate_pub;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;
    // dynamic_server
    // dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> dynamic_server;
    dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> *dsrv_;

    visualization_msgs::Marker points, line_strip, goal_circle;
    ackermann_msgs::AckermannDrive ackermann_cmd;
    geometry_msgs::Point odom_goal_pos, nav_goal_pos_forward, nav_goal_pos_back;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path,last_map_path;

    double L,Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
    double gas_gain, base_angle, base_speed_forward, base_speed_back, angle_gain_forward, angle_gain_back, goal_radius;
    int controller_freq, sub_locate_data;
    bool foundForwardPt, foundBackPt, goal_received,path_received, goal_reached, AllowReverse;

    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void locateCB(const std_msgs::Int32 &data);
    void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void hypathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    void goalReachingCB(const ros::TimerEvent &);
    void controlLoopCB(const ros::TimerEvent &);
    void dynamicCB(qingzhou_nav::L1_dynamicConfig &config, uint32_t level);

}; // end of class

L1Controller::L1Controller()
{
    // Private parameters handler
    ros::NodeHandle pn("~");

    // Publishers and Subscribers
    //  /move_base/NavfnROS/plan
    //  /move_base_hy/HybridAStarPlanner/plan
    locate_sub = n_.subscribe("/qingzhou_locate", 1, &L1Controller::locateCB, this);
    odom_sub = n_.subscribe("/odom_ekf", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    hypath_sub = n_.subscribe("/move_base_hy/HybridAStarPlanner/plan", 1, &L1Controller::hypathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("qz_cmd_vel_l1", 1);
    planner_pub = n_.advertise<nav_msgs::Path>("/planner_global_l1", 1);
    locate_pub = n_.advertise<std_msgs::Int32>("/qingzhou_locate", 1);

    // Timer
    timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq), &L1Controller::controlLoopCB, this);  // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    // dynamic_server

    // dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> dynamic_server;
    // dynamic_server.setCallback(boost::bind(&L1Controller::dynamicCB,this,_1,_2));
    dsrv_ = new dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig>(pn);
    dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig>::CallbackType cb = boost::bind(&L1Controller::dynamicCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // Init variables
    Lfw = goal_radius = getL1Distance(Vcmd);
    foundForwardPt = false;
    foundBackPt = false;
    goal_received = false;
    goal_reached = false;
    path_received = false;
    ackermann_cmd.speed = 0.0;
    ackermann_cmd.steering_angle = 0.0;

    // Show info
    ROS_INFO("--------------------L1_controller is ready--------------------------");

    // Visualization Marker Settings
    initMarker();
}

L1Controller::~L1Controller()
{
    delete dsrv_;
}

void L1Controller::locateCB(const std_msgs::Int32 &data)
{
    sub_locate_data = data.data;
}

void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom = *odomMsg;
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    if (sub_locate_data != 11 && sub_locate_data != 12 && sub_locate_data != 13)
    {
        last_map_path = *pathMsg;
        // ROS_WARN("--------------------------size:%d------------------", last_map_path.poses.size());
        if (last_map_path.poses.size() > 30)
        {
            path_received = true;
            map_path = last_map_path;
        }
    }
}

void L1Controller::hypathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    if (sub_locate_data == 11 || sub_locate_data == 12 || sub_locate_data == 13)
    {
        last_map_path = *pathMsg;
        if (last_map_path.poses.size() > 10)
        {
            path_received = true;
            map_path = last_map_path;
        }
    }
}

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
    goal = *goalMsg;
    goal_ = *goalMsg;
    try
    {
        for (int i = 0; i < 6; ++i)
        {
            if (goal.pose.position.x > goalPoint.x1[i] && goal.pose.position.x < goalPoint.x2[i] &&
                goal.pose.position.y > goalPoint.y1[i] && goal.pose.position.y < goalPoint.y2[i])
            {
                goalLocation = RobotLocation(i);
                if (i == 5)
                {
                    goalLocation = Reverse;
                }
                // ROS_INFO("%d------in the callback", goalLocation);
                break;
            }
        }
        switch (goalLocation)
        {
        case Start:
            goal.pose.position.x += (1 * Lfw);
            break;
        case Load:
            goal.pose.position.y -= (1 * Lfw);
            // ROS_INFO("---------------------------goal of Load---------------------------[L1]");
            break;
        case TrafficLight:
            goal.pose.position.y -= (1 * Lfw);
            break;
        case Unload:
            goal.pose.position.y += (1 * Lfw);
            break;
        case RoadLine:
            goal.pose.position.y += (1 * Lfw);
            break;
        case Reverse:
            goal.pose.position.x -= (1 * Lrv);
            // ROS_INFO("---------------------------goal of AllowReverse---------------------------[L1]");
            break;
        default:
            ROS_INFO("---------------------------goal of Default---------------------------[L1]");
            break;
        }
        // ROS_INFO("Distance too short");
        goal_received = true;
        goal_reached = false;

        geometry_msgs::PoseStamped odom_goal;

        tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        // ROS_INFO("x: %f, y: %f", nav_goal_pos.x, nav_goal_pos.y);

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose &carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp, yaw;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp, tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

    if (car_car2wayPt_x > 0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

bool L1Controller::isWayPtAwayFromLfw_LrvDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos, double Lfw_Lrv)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < Lfw_Lrv)
        return false;
    else if (dist >= Lfw_Lrv)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    double distance, distance_x, distance_y;
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point backPt;
    geometry_msgs::Point judgePt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;
    foundBackPt = false;
    bool _isForwardWayPt;
    bool _isWayPtAwayFromLfw_LrvDist;

    if (!goal_reached)
    {
        double y_limit = -7.57; // 7.58对应0.4的膨胀半径

        for (int j = 0; j < map_path.poses.size(); j++)
        {
            if (map_path.poses[j].pose.position.y < y_limit)
            {
                map_path.poses[j].pose.position.y = y_limit;
            }
        }                              // planner_pub
        planner_pub.publish(map_path); // 可视化操作

        for (int i = 0; i < map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            // 坡道限幅
            if (map_path_pose.pose.position.y < y_limit)
            {
                map_path_pose.pose.position.y = y_limit;
            }

            geometry_msgs::PoseStamped odom_path_pose;
            try
            {
                tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map", odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

                if (AllowReverse)
                {
                    if (!_isForwardWayPt)
                    {
                        _isWayPtAwayFromLfw_LrvDist = isWayPtAwayFromLfw_LrvDist(odom_path_wayPt, carPose_pos, Lrv);
                        if (_isWayPtAwayFromLfw_LrvDist)
                        {
                            backPt = odom_path_wayPt;
                            foundForwardPt = false;
                            foundBackPt = true;
                            break;
                        }
                    }
                }
                else
                {
                    if (_isForwardWayPt)
                    {
                        _isWayPtAwayFromLfw_LrvDist = isWayPtAwayFromLfw_LrvDist(odom_path_wayPt, carPose_pos, Lfw);
                        if (_isWayPtAwayFromLfw_LrvDist)
                        {
                            forwardPt = odom_path_wayPt;
                            foundForwardPt = true;
                            foundBackPt = false;
                            break;
                        }
                    }
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        //
        if (AllowReverse)
        {
            if (!(_isWayPtAwayFromLfw_LrvDist) && !(_isForwardWayPt))
            // if (!_isForwardWayPt)
            {
                geometry_msgs::PoseStamped goal_;
                tf_listener.transformPose("odom", ros::Time(0), goal, "map", goal_);
                // ROS_INFO("Distance too short");
                nav_goal_pos_back.x = goal_.pose.position.x;
                nav_goal_pos_back.y = goal_.pose.position.y;
                nav_goal_pos_back.z = 0.0;
                // ROS_INFO("Distance too short");
                backPt = nav_goal_pos_back;
                foundForwardPt = false;
                foundBackPt = true;
            }
            else
            {
                // ROS_INFO("--------------No back point--------------");
            }
        }
        else
        {
            if (!(_isWayPtAwayFromLfw_LrvDist) && _isForwardWayPt)
            // if (_isForwardWayPt)
            {
                geometry_msgs::PoseStamped goal_;
                tf_listener.transformPose("odom", ros::Time(0), goal, "map", goal_);
                nav_goal_pos_forward.x = goal_.pose.position.x;
                nav_goal_pos_forward.y = goal_.pose.position.y;
                nav_goal_pos_forward.z = 0.0;

                forwardPt = nav_goal_pos_forward;
                foundForwardPt = true;
                foundBackPt = false;
            }
            else
            {
                // ROS_INFO("--------------No forward point--------------");
            }
        }
        if (foundBackPt)
        {
            judgePt = backPt;
        }
        else if (foundForwardPt)
        {
            judgePt = forwardPt;
        }
    }
    else if (goal_reached)
    {
        forwardPt = odom_goal_pos;
        backPt = odom_goal_pos;
        judgePt = odom_goal_pos;
        foundForwardPt = false;
        foundBackPt = false;
        // ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if (foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw) * (judgePt.x - carPose_pos.x) + sin(carPose_yaw) * (judgePt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw) * (judgePt.x - carPose_pos.x) + cos(carPose_yaw) * (judgePt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

// 返回前瞻目标点在车坐标系内的角度
double L1Controller::getEta(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    double eta;
    if (AllowReverse)
    {
        eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x - lrv);
    }
    else
    {
        eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
    }
    return eta;
}

double L1Controller::getCar2GoalDist()
{
    geometry_msgs::PoseStamped temp_goal_;
    tf_listener.transformPose("odom", ros::Time(0), goal_, "map", temp_goal_);
    odom_goal_pos = temp_goal_.pose.position;

    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double &_Vcmd)
{
    double L1 = 0;
    if (_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if (_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd * 2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta, double Lfw_Lrv, double lfw_lrv)
{
    // double steering_angle = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    // ROS_INFO("ETA = %.2f", -eta*180.0/PI);
    double steering_angle = atan2((L * sin(eta)), (Lfw_Lrv / 2 + lfw_lrv * cos(eta)));
    // ROS_INFO("Steering Angle = %.2f", steering_angle);
    return steering_angle;
}

double L1Controller::getGasInput(const float &current_v)
{
    double u = (Vcmd - current_v) * gas_gain;
    // ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}

void L1Controller::goalReachingCB(const ros::TimerEvent &)
{

    double temp_dist;
    if (goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if (AllowReverse)
        {
            temp_dist = 0.4;//0.3
        }
        else if (sub_locate_data == TrafficLightToUnload || sub_locate_data == Unload)
        {
            temp_dist = 0.5;
        }
        else
        {
            temp_dist = 0.59;
        }
        if (car2goal_dist < temp_dist)
        {
            goal_reached = true;
            goal_received = false;
            path_received = false;
            if (sub_locate_data == RoadLineToReverse)
            {
                // usleep(250000); // 延时0.25秒
                std_msgs::Int32 temp_data;
                temp_data.data = int(Reverse);
                locate_pub.publish(temp_data);
                ROS_INFO("--------------------------更新Reverse位置---------------------------");
            }
            else if (sub_locate_data == Reverse)
            {
                // usleep(100000);//延时0.1秒
                std_msgs::Int32 temp_data;
                temp_data.data = int(ReverseToStart);
                locate_pub.publish(temp_data);
                ROS_INFO("--------------------------更新ReverseToStart位置---------------------------");
            }
            ROS_INFO("[L1]-----------------------Goal Reached !-----------------------");
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent &)
{

    geometry_msgs::Pose carPose = odom.pose.pose;
    ackermann_cmd.speed = 0;
    ackermann_cmd.steering_angle = 0;

    if (goal_received && path_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose); // 返回前瞻目标点在车坐标系内的角度
        if (foundForwardPt)
        {
            ackermann_cmd.steering_angle = getSteeringAngle(eta, Lfw, lfw) * angle_gain_forward;
            /*Estimate Gas Input*/
            if (!goal_reached)
            {
                ackermann_cmd.speed = base_speed_forward;
            }
        }
        else if (foundBackPt && AllowReverse)
        {
            ackermann_cmd.steering_angle = -getSteeringAngle(eta, Lrv, lrv) * angle_gain_back;
            if (!goal_reached)
            {
                ackermann_cmd.speed = -base_speed_back;
            }
        }

        if (sub_locate_data == 9)
        {
            ackermann_cmd.steering_angle += PI / 60; // 第四段手动加3度的偏移
        }
        else if (sub_locate_data == 7)
        {
            ackermann_cmd.steering_angle += PI / 30;
        }
    }
    pub_.publish(ackermann_cmd);
}

void L1Controller::dynamicCB(qingzhou_nav::L1_dynamicConfig &config, uint32_t level)
{
    L = config.L;
    Lrv = config.Lrv;
    Vcmd = config.Vcmd;
    lfw = config.lfw;
    lrv = config.lrv;
    Lfw = config.Lfw;
    AllowReverse = config.AllowReverse;

    controller_freq = config.controller_freq;
    angle_gain_forward = config.angle_gain_forward;
    angle_gain_back = config.angle_gain_back;
    gas_gain = config.gas_gain;
    base_speed_forward = config.base_speed_forward;
    base_speed_back = config.base_speed_back;
    base_angle = config.base_angle;
    ROS_INFO("-----params have been changed----[L1]");
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    // Initiate ROS
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "L1Controller_v3");
    L1Controller controller;
    ros::spin();
    return 0;
}