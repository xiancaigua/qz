#include <ros/ros.h>
#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/Twist.h"


class Transformer
{
public:
    Transformer(ros::NodeHandle nh);
    ~Transformer();

    void run();
    void Transform(const geometry_msgs::Twist::ConstPtr &msg_p);

public:
    ros::Subscriber sub_dwa;
    ros::Publisher pub_cmd;
    ackermann_msgs::AckermannDrive cmd;
    bool pub_flag;
};
