#include "test_pkg/msg_transform.h"



int    main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "msg_transform");

    ros::NodeHandle nh; // 该类封装了 ROS 中的一些常用功能

    Transformer node(nh);

    node.run();
    return 0;
}
Transformer::Transformer(ros::NodeHandle nh)
{
    sub_dwa = nh.subscribe("/cmd_vel", 1, &Transformer::Transform, this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDrive>("qz_cmd_vel_l1", 2);
    pub_flag = false;

}

void Transformer::Transform(const geometry_msgs::Twist::ConstPtr &msg_p)
{
    double x = msg_p->linear.x;
    double theta = msg_p->angular.z;
    cmd.speed = x;
    cmd.steering_angle = theta;
    pub_flag = true;
}

void Transformer::run()
{
while (ros::ok())
{
    ros::spinOnce();
    if (pub_flag){
            pub_cmd.publish(cmd);
        }
        else{
            ROS_INFO("[Transformer]-------No cmd from DWA");
        };

}
}

Transformer::~Transformer(){

}

