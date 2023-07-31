/*********************************
****注释是胡杨写的，胡杨！yyds！****
*********************************/

/*
程序流程：
    初始化
    收目标点
    看局部规划
    局部规划未寄就去
    到了就回到第二步
*/

/*-----------------------------------------------------------

一、初始化部分

    1. 首先，实例化L1Controller这一类，运行构造函数，随后进入spin

    2. 构造函数中，订阅话题，发布话题，创建定时器，设置初始动态参数
       创建动态参数服务器，将服务器挂到~下，也就是/L1Controller_v3
       下

    3. 设置参数服务器的回调函数（修改小车运动控制相关参数）

    4. 初始化Visualization Marker，对它的一些参数进行设置（是用于
    |  rviz中可视化的功能包）
    |--4.1 Marker的header.frame_id设置为odom，用于tf，说明这些
    |      物体的坐标都是在odom坐标系下的
    |--4.2 Marker的ns，namespace设置为“Markers”A
    |--4.3 Marker的action设置为Add，值为0
    |--4.4 Marker位置的四元数的w设置为1，即x正向
    |--4.5 Marker变量的id设置，和ns一起构成一个物体的名字
    |--4.6 Marker变量的type为Points，值为8，不知道是什么形状
    |      其他两个的形状分别是线条和圆柱体
    |--4.7 设置Marker的几何体的大小
    |--4.8 设置Marker的几何体的颜色
    | 发布Marker的信息再marker_pub那

二、订阅话题回调函数部分

    1. odom_sub
    |--1.1 将/odom_ekf的信息存到odom变量里面

    2. path_sub
    |--2.1 将/move_base/NavfnROS/plan规划的路径信息存map_path变
    |      量里面

    3. goal_sub
    |  回调函数用到C++异常处理（tf比较容易出错）
    |--3.1 将/move_base_simple/goal里的目标点坐标角度存
    |      odom_goal_pos变量里面
    |--3.2 transformPose()将rviz中发布的在map坐标系下的目标点坐
    |      标变换到在odom坐标系下的坐标，并存入odom_goal变量中，
    |      这么做的原因可能是由于需要使用到odom_ekf发布的小车位置
    |      信息来判断小车是否到达目标点，而这是在odom坐标系下的坐
    |      标信息，为了方便，少做一次tf变换，所以让目标点的坐标信
    |      息也取odom坐标系下的坐标信息（这样可以少做很多tf变换，
    |      节约资源）
    |--3.3 给odom_goal_pos和goal_circle.pose变量赋值，设置状态为
    |      收到目标和未抵达（改goal_received，goal_reached）
    |--3.4 发布有关goal在rviz中的可视化信息
    | 如果以上过程出错，则输出错误信息，程序暂停运行1秒

三、定时器中断回调函数

    1. controlLoopCB
    |  计算并发布小车运动信息/ackermann_cmd
    |--1.1 读小车在odom坐标系下的位置和角度信息，将速度和转角初始化
    |--1.2 如果接收到了目标点，则调用getEta函数（四、4）获取小车前
    |      往的下一个点在小车坐标系下的方向
    |--1.3 如果有找到下一个前往的点（调用getEta函数时有判断），通过
    |      调用getSteeringAngle（四、7）
    |--1.4 发布ackermann_cmd信息，控制小车运动


    2. goalReachingCB
    |  收到目标点，开始调用getCar2GoalDist函数（四、5）算距离，距离
    |  小于0.2，就认为到了目标点，改goal_received，goal_reached

四、L1Controller类中public函数

    1. isForwardWayPt
    |  通过计算路径点在小车坐标系x方向的正负，来判断路径点是否在前方
    |  在前方，返回true
    |  计算公式见具体函数实现

    2. isWayPtAwayFromLfwDist
    |  通过路径点和小车在odom坐标系下的坐标，用勾股定理计算路径点和
    |  小车的距离，判断是否满足最大距离要求

    3. getYawFromPose
    |  将四元数转换成欧拉角，并返回欧拉角的Yaw角数据（普通）

    4. getEta
    |  输入小车坐标，调用get_odom_car2WayPtVec函数（四、9），使用到
    |  map_path等数据，得到小车将要前往的下一个点在小车坐标系中的角度

    5. getCar2GoalDist
    |  用勾股定理在odom坐标系下计算小车和目标点的距离

    6. getL1Distance
    |  通过某种方法来给Lfw，和goal_radius赋值，也就是确定局部路径的
    |  目标点的半径范围，goal_radius是rviz可视化的参数，最终目标点
    |  在rviz上的大小

    7. getSteeringAngle
    |  结合车长L，Lfw，lfw，eta计算轮胎的转角，弧度转角度
    |  -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI)
    |  不知道为什么要这样算----------

    8. getGasInput
    |  结合小车当前的速度，以及小车的目标速度，得到小车当前的加速度，
    |  可能是想用这个来调小车的速度，实现加速的过程，但是实现的地方被
    |  注释了，也没有写完，就没用，useless

    9. get_odom_car2WayPtVec
    |  得到局部路径下一个点在小车坐标系的坐标
    |--9.1 读取小车在odom坐标系下的坐标，并通过getYawFromPose（四、3）
    |      四元数计算得到小车在odom坐标系下的z轴转角Yaw
    |--9.2 如果小车未到目标点（修改于：三、2），则进入一个循环，遍
    |      历规划的路径上的每一个点
    |--9.3 将路径点从map坐标系变换到odom坐标系，调用isForwardWayPt
    |      函数（四、1）判断路径点是否在小车前方
    |--9.4 路径点在小车前方，继续判断该路径点是否在最短路径点跟随范
    |      围内，调用isWayPtAwayFromLfwDist函数（四、2），若在范
    |      围内，就结束遍历，有可前往的路径点
    |--9.5 有可前往的路径点，先进行rviz可视化处理（先清空所有元素，
    |      再往里面添加起点和去的点，连成一条线），再计算前往的点在
    |      小车坐标系的x，y

五、运动控制参数

    1.  L；		车长，影响车转弯控制（getSteeringAngle函数）
    2.  lrv：	没用到，不知道是什么
    3.  Lrv：	没用到，不知道是什么
    4.  Vcmd：	可能是控制的目标速度，体现不出功能，可能代码还没写
                （暂时和base_speed一致？）
    5.  Lfw：	局部规划目标点半径，从功能上可视为车掉头时最小直径
                其值受到Vcmd的值的影响，不在动态参数服务器内（maybe）
    6.  lfw：	不知道什么意思，影响车转弯控制（getSteeringAngle函数）
    7.  controller_freq：	发布ackermann信息控制小车的频率
    8.  angle_gain：		控制车转弯速度，轮胎的转角
    9.  gas_gain			控制小车加速度，没有实现，没用
    10. base_speed			控制小车行驶的速度
    11. base_angle			没有用到，没用

六、动态参数解析

    都在头文件里面了，可以看

-----------------------------------------------------------*/

/*
评价：

它的局部规划的最大半径我认为没有用，应该直接判断全局规划的第一个点，
或者前几个点，判断是不是在屁股后面，在的话就是不能走，有的时候，从装
货区直接发点到卸货区，全局规划从S弯那走，但是绕了一圈的路径点是会有点
在那个半径内而且在前方的，会让车误以为可以走啊，然后就在装货区直接右
拐撞到墙上，就是写的不合理啊

但是这个路径规划虽然只算了控制的角度，速度完全不管，很简单，却非常的
有用啊，资源占用也比较小
*/

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

#define PI 3.14159265358978

/********************/
/* CLASS DEFINITION */
/********************/

struct GoalPoint
{
    float x1[5];
    float y1[5];
    float x2[5];
    float y2[5];
    GoalPoint()
    {
        x1[0] = -0.5;
        y1[0] = -0.5;
        x2[0] = 0.5;
        y2[0] = 0.5;
        x1[1] = 1.7;
        y1[1] = -3.9;
        x2[1] = 2.7;
        y2[1] = -2.9;
        x1[2] = 1.7;
        y1[2] = -6.3;
        x2[2] = 2.7;
        y2[2] = -5.3;
        x1[3] = -2.7;
        y1[3] = -6.5;
        x2[3] = -1.7;
        y2[3] = -5.5;
        x1[4] = 0.5;
        y1[4] = -4.1;
        x2[4] = 1.5;
        y2[4] = -2.7; //-3.0
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

    ros::Subscriber odom_sub, path_sub, goal_sub, hypath_sub, locate_sub;
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
    nav_msgs::Path map_path, odom_path;

    double L,
        Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
    double gas_gain, base_angle, base_speed_forward, base_speed_back, angle_gain_forward, angle_gain_back, goal_radius;
    int controller_freq, sub_locate_data, index;
    bool foundForwardPt, foundBackPt, goal_received, goal_reached, AllowReverse;

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
    ackermann_cmd.speed = 0.0;
    ackermann_cmd.steering_angle = 0.0;

    // Show info
    ROS_INFO("[param] base_speed: %f", base_speed_forward);
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] angle_gain: %f", angle_gain_forward);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

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
    if (AllowReverse)
    {
        odom = *odomMsg;
        odom.pose.pose.position.x -= (lrv + lfw);
    }
    else
    {
        odom = *odomMsg;
    }
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    if (sub_locate_data != 11 && sub_locate_data != 12 && sub_locate_data != 13)
    {
        map_path = *pathMsg;
    }
}

void L1Controller::hypathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    // ROS_INFO("--------------------------hy map path callback:%d---------------------------", sub_locate_data);
    if (sub_locate_data == 11 || sub_locate_data == 12 || sub_locate_data == 13)
    {
        map_path = *pathMsg;
        // ROS_INFO("--------------------------hy map path---------------------------");
    }
}

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
    geometry_msgs::PoseStamped goal = *goalMsg;
    try
    {
        for (int i = 0; i < 5; ++i)
        {
            if (goal.pose.position.x > goalPoint.x1[i] && goal.pose.position.x < goalPoint.x2[i] &&
                goal.pose.position.y > goalPoint.y1[i] && goal.pose.position.y < goalPoint.y2[i])
            {
                goalLocation = RobotLocation(i);
                // ROS_INFO("%d------in the callback", goalLocation);
                break;
            }
        }

        goal_received = true;
        goal_reached = false;
        geometry_msgs::PoseStamped odom_goal;
        switch (goalLocation)
        {
        case Start:
            goal.pose.position.x += (2 * Lfw);
            // ROS_INFO("---------------------------goal of start---------------------------[L1]");
            break;
        case Load:
            goal.pose.position.y -= (2 * Lfw);
            // ROS_INFO("---------------------------goal of Load---------------------------[L1]");
            break;
        case TrafficLight:
            goal.pose.position.y -= (2 * Lfw);
            // ROS_INFO("---------------------------goal of Traffic---------------------------[L1]");
            break;
        case Unload:
            goal.pose.position.y += (2 * Lfw);
            // ROS_INFO("---------------------------goal of Unload---------------------------[L1]");
            break;
        case RoadLine:
            goal.pose.position.y += (2 * Lfw);
            ROS_INFO("---------------------------goal of RoadLine---------------------------[L1]");
            break;
        default:
            if (AllowReverse && sub_locate_data == int(Reverse))
            {
                goal.pose.position.x -= (2 * Lfw);
                ROS_INFO("---------------------------goal of AllowReverse---------------------------[L1]");
            }
            else
            {
                goal.pose.position.x += (2 * Lfw);
                ROS_INFO("---------------------------goal of default---------------------------[L1]");
            }
            break;
        }

        tf_listener.transformPose("odom", ros::Time(0), goal, "map", odom_goal);
        odom_goal_pos = odom_goal.pose.position;

        nav_goal_pos_forward.x = odom_goal_pos.x;
        nav_goal_pos_forward.y = odom_goal_pos.y;
        nav_goal_pos_forward.z = 0.0;

        nav_goal_pos_back.x = odom_goal_pos.x;
        nav_goal_pos_back.y = odom_goal_pos.y;
        nav_goal_pos_back.z = 0.0;

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

    // float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y - lrv;
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

    // if(dist < 0.6)
    //     return false;
    // else if(dist >= 0.6)
    //     return true;

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
        double y_limit = -7.56; // 7.58

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
                // ROS_INFO("Distance too short");
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
    double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
    return eta;
}

double L1Controller::getCar2GoalDist()
{
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
        // ROS_INFO("[L1]---------------------距离：%f----------------------", car2goal_dist);
        if (AllowReverse)
        {
            temp_dist = 0.15; // 0.15
        }
        else
        {
            temp_dist = 0.55;
        }
        if (car2goal_dist < temp_dist)
        {
            goal_reached = true;
            goal_received = false;
            if (sub_locate_data == int(RoadLineToReverse))
            {
                std_msgs::Int32 temp_data;
                temp_data.data = int(Reverse);
                locate_pub.publish(temp_data);
                ROS_INFO("--------------------------更新Reverse位置---------------------------");
            }
            else if (sub_locate_data == int(Reverse))
            {
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
    geometry_msgs::Twist carVel = odom.twist.twist;
    ackermann_cmd.speed = 0;
    ackermann_cmd.steering_angle = 0;

    if (goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose); // 返回前瞻目标点在车坐标系内的角度
        if (foundForwardPt)
        {
            // ROS_INFO("-------------------foundForwardPt----------------------");
            ackermann_cmd.steering_angle = getSteeringAngle(eta, Lfw, lfw) * angle_gain_forward;
            /*Estimate Gas Input*/
            if (!goal_reached)
            {
                ackermann_cmd.speed = base_speed_forward;
            }
        }
        else if (foundBackPt && AllowReverse)
        {
            // ROS_INFO("-------------------------------foundBackPt----------------------------");
            ackermann_cmd.steering_angle = -getSteeringAngle(eta, Lrv, lrv) * angle_gain_back;
            if (!goal_reached)
            {
                ackermann_cmd.speed = -base_speed_back; // 先写死，可以加到动态参数里
            }
        }
        // ROS_INFO("-----------------------NoPt-----------------------");
    }
    if (goal_received || goal_reached)
    {
        pub_.publish(ackermann_cmd);
    }
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
    ROS_INFO("-----params have been changed----");
    ROS_INFO("base_speed:%f", base_speed_forward);
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