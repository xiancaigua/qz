#include"test_pkg/cmd_filter.h"

void locateCB(const std_msgs::Int32::ConstPtr &msg)
{
    robotlocation = *msg;
}

void l1_CB(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_from_l1 = *msg;
}

void vision_CB(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_from_vision = *msg;
}

void pub_CB(const ros::TimerEvent&)
{
    if (robotlocation.data == int(RoadLine))
    {
        ROS_INFO("-----------------------cmd_from_vision-----------------------------");
        final_cmd = cmd_from_vision;
    }
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
    cmd_vel_l1_sub = nh.subscribe<geometry_msgs::Twist>("/qz_cmd_vel_l1", 1, l1_CB);
    cmd_vel_vision_sub = nh.subscribe<geometry_msgs::Twist>("/qz_cmd_vel_vision", 1, vision_CB);

    qz_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/qz_cmd_vel", 1);

    timer = nh.createTimer(ros::Duration((1.0) / 10),pub_CB ); // Duration(0.05) -> 20Hz

    ros::spin();
    return 0;
}