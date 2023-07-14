#include"test_pkg/cmd_filter.h"

void locateCB(const std_msgs::Int32::ConstPtr &msg)
{
    robotlocation = *msg;
}

void l1_CB(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    cmd_from_l1 = *msg;
}

void vision_CB(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    cmd_from_vision = *msg;
}

void dwa_cmd_CB(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    cmd_from_dwa = *msg;
}

void dwaCB(const std_msgs::Int32::ConstPtr &msg)
{
    dwa_flag = *msg;
}

void pub_CB(const ros::TimerEvent&)
{
    // &&cmd_from_vision.linear.x != 0  && dwa_flag.data == 0
    if (robotlocation.data == int(RoadLine) && cmd_from_vision.speed != 0)
    {
        ROS_INFO("-----------------------cmd_from_vision-----------------------------");
        final_cmd = cmd_from_vision;
    }
    // else if (robotlocation.data == int(RoadLine) && dwa_flag.data == 1)
    // {
    //     final_cmd = cmd_from_dwa;
    //     ROS_INFO("-----------------------cmd_from_dwa-----------------------------");
    // }
    else
    {
        final_cmd = cmd_from_l1;
    }

    qz_cmd_vel_pub.publish(final_cmd);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "qz_cmd_filted_node");
    ros::NodeHandle nh;
    // ROS_INFO("ackermann_cmd_filter Finish");
    location_sub = nh.subscribe<std_msgs::Int32>("/qingzhou_locate", 1, locateCB);
    filter_dwa_flag_sub = nh.subscribe<std_msgs::Int32>("/filter_dwa_flag", 1, dwaCB);
    cmd_vel_dwa_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/dwa_cmd_vel", 1, dwa_cmd_CB);
    cmd_vel_l1_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/qz_cmd_vel_l1", 1, l1_CB);
    cmd_vel_vision_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/qz_cmd_vel_vision", 1, vision_CB);

    qz_cmd_vel_pub = nh.advertise<ackermann_msgs::AckermannDrive>("/qz_cmd_vel", 1);

    timer = nh.createTimer(ros::Duration((1.0) / 10),pub_CB ); // Duration(0.05) -> 20Hz

    ros::spin();
    return 0;
}