#include "test_pkg/qingzhou_bringup.h"

long long LeftticksPerMeter = 0;
long long rightticksPerMeter = 0;
long long LeftticksPer2PI = 0;
long long rightticksPer2PI = 0;

actuator::actuator(ros::NodeHandle nh)
{
    m_baudrate = 115200;         // 114514 哼哼哼，阿啊啊啊啊啊
    m_serialport = "/dev/stm32"; // 这个在lsusb  配置软连接时有

    calibrate_lineSpeed = 0;    // 后面launch文件设置参数服务器
    calibrate_angularSpeed = 0; // 后面launch文件设置参数服务器
    batteryVoltage = 0;
    ticksPerMeter = 0; // 后面launch文件设置参数服务器
    ticksPer2PI = 0;   // 后面launch文件设置参数服务器
    linearSpeed = 0;   // 来自编码器，用于发布在odom中计算线速度
    angularSpeed = 0;  // 同上
    velDeltaTime = 0;  // linearSpeed = detdistance/velDeltaTime;  数值上来自读取stm32的时间差
    traffic_flag = 1;
    amcl_y = 0.0;

    zero_cmd.TargetSpeed = 0;
    zero_cmd.TargetAngle = 0;

    nh.param("mcubaudrate", m_baudrate, m_baudrate);
    nh.param("mcuserialport", m_serialport, std::string("/dev/stm32"));
    nh.param("calibrate_lineSpeed", calibrate_lineSpeed, calibrate_lineSpeed);
    nh.param("calibrate_angularSpeed", calibrate_angularSpeed, calibrate_angularSpeed);
    nh.param("ticksPerMeter", ticksPerMeter, ticksPerMeter);
    nh.param("ticksPer2PI", ticksPer2PI, ticksPer2PI);

    try
    {

        // 准备开启串口
        std::cout << "[qingzhou_actuator-->]"
                  << "Serial initialize start!" << std::endl;
        ser.setPort(m_serialport.c_str());
        ser.setBaudrate(m_baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(30);
        ser.setTimeout(timeout);
        ser.open();
        // 串口开启完毕
    }
    catch (serial::IOException &e)
    {
        std::cout << "[qingzhou_actuator-->]"
                  << "Unable to open port!" << std::endl;
    }
    if (ser.isOpen())
    {
        std::cout << "[qingzhou_actuator-->]"
                  << "Serial initialize successfully!" << std::endl;
    }
    else
    {
        std::cout << "[qingzhou_actuator-->]"
                  << "Serial port failed!" << std::endl;
    }
    // debug 串口完成

    // 读取ROS中的运动控制话题，用于解析并且组织到下位机,这是具有经过filter分滤过的
    sub_l1 = nh.subscribe("/qz_cmd_vel_l1", 1, &actuator::l1_move_callback, this);
    sub_vision = nh.subscribe("/qz_cmd_vel_vision", 1, &actuator::vision_move_callback, this);
    sub_traffic = nh.subscribe("/traffic_flag", 1, &actuator::traffic_move_callback, this);
    location_sub = nh.subscribe("/qingzhou_locate", 1, &actuator::locateCB, this);
    SubOdomReset = nh.subscribe("/OdomReset", 1, &actuator::OdomReset, this);
    amcl_sub = nh.subscribe("/amcl_pose",1,&actuator::amcl_callback, this);
    // 这一行是调试用，记得删掉

    // 发布下位机IMU  ODOM  电池数据到ROS 话题中
    pub_imu = nh.advertise<sensor_msgs::Imu>("raw", 2);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 5);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 2);
    pub_battery = nh.advertise<std_msgs::Float32>("battery", 10);

    odomreset.data = 1;
}

actuator::~actuator()
{
}

// filter 's callback
void actuator::l1_move_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    memset(&l1_cmd, 0, sizeof(sMartcarControl)); // 清零movebase数据存储区

    float v = msg->speed;
    float w = msg->steering_angle;

    l1_cmd.TargetSpeed = v * 42 / 0.43;              // 这里可能要加一个增益值,这里是我随意写的
    l1_cmd.TargetAngle = round(atan(w * CARL / v) * 57.3); // 数学原理不清楚，保持与官方一致
    l1_cmd.TargetAngle += 60;
    // moveBaseControl.TargetAngle += 0.017435* 57.3;
    // 0.0175左偏，0.017435右偏
    //  ROS_INFO("%.2f", &v);
    //  ROS_INFO("%.2f", &w);
    //  ROS_INFO("----targetangle----%.2f", &moveBaseControl.TargetAngle);
}

void actuator::vision_move_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    memset(&vision_cmd, 0, sizeof(sMartcarControl)); // 清零movebase数据存储区

    float v = msg->speed;
    float w = msg->steering_angle;

    vision_cmd.TargetSpeed = v * 42 / 0.43;             // 这里可能要加一个增益值,这里是我随意写的，原来是乘32
    // vision_cmd.TargetAngle = round(atan(w * CARL / v)); // 数学原理不清楚，保持与官方一致round(atan(w*CARL/v)*57.3);
    vision_cmd.TargetAngle = round(atan(w * CARL / v) * 57.3); // 数学原理不清楚，保持与官方一致round(atan(w*CARL/v)*57.3);
    vision_cmd.TargetAngle += 60;
}

void actuator::traffic_move_callback(const std_msgs::Int32::ConstPtr &msg)
{
    traffic_flag = (*msg).data;
}

void actuator::locateCB(const std_msgs::Int32::ConstPtr &msg)
{
    robotlocation = *msg;
}

void actuator::OdomReset(const std_msgs::Int32::ConstPtr &msg)
{
    odomreset = *msg;
}

void actuator::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg)
{
    amcl_y = (*amclMsg).pose.pose.position.y;
}

void actuator::run()
{
    memset(&moveBaseControl, 0, sizeof(sMartcarControl)); // 清零movebase数据存储区
    memset(&l1_cmd, 0, sizeof(sMartcarControl));          // 清零movebase数据存储区
    memset(&vision_cmd, 0, sizeof(sMartcarControl));      // 清零movebase数据存储区

    ros::Rate rate(150);

    double x = 0.0; // 后面发布在odom中会用，用速度计算距离，随后加在odom里
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;

    while (ros::ok())
    {
        ros::spinOnce();
        memset(&moveBaseControl, 0, sizeof(sMartcarControl)); // 清零movebase数据存储区

        ROS_INFO("[BRING_UP]---------------%f,%d", amcl_y, traffic_flag);
        // ROS_INFO("[BRING_UP]---------------Bring Up is Ready");
        if (robotlocation.data == int(RoadLine) )
        {
            moveBaseControl = vision_cmd;
            // ROS_INFO("[BRING_UP]---------------FROM VISION");
        }
        else if (amcl_y < -5.1 && traffic_flag == 0)
        {
            moveBaseControl = zero_cmd;
            // ROS_INFO("----%d",int(moveBaseControl.speed));
            ROS_INFO("[BRING_UP]---------------FROM ZERO");
        }
        else
        {
            moveBaseControl = l1_cmd;
        }
        if (odomreset.data == 0){
            memset(&moveBaseControl, 0, sizeof(sMartcarControl)); // 清零movebase数据存储区
        }
        // 计算velDeltatime,后面算速度用得到
        current_time = ros::Time::now();
        velDeltaTime = (current_time - last_time).toSec();
        last_time = ros::Time::now();

        recvCarInfoKernel();
        pub_9250(); // 你不必知道发生了什么，不重要，发布电池相关数据而以
        int bit0_z = batteryVoltage & (255);
        int bit1_x = (unsigned char)((batteryVoltage) >> 8);
        currentBattery.data = bit0_z + ((float)bit1_x / 100.0f);
        pub_battery.publish(currentBattery);

        // 下面是用编码器数据计算速度用以发布在odom坐标系中
#if 10
        if (encoderLeft > 220 || encoderLeft < -220)
        {
            encoderLeft = 0;
            ROS_INFO(">220-LEFT--66678r97836r97");
        };
        if (encoderRight > 220 || encoderRight < -220)
        {
            encoderRight = 0;
            ROS_INFO(">220--RIGHT-66678r97836r97");
        };

        // encoderLeft = -encoderLeft;
        encoderRight = -encoderRight;

        detEncode = (encoderLeft + encoderRight) / 2;
        detdistance = detEncode / ticksPerMeter;
        detth = (encoderRight - encoderLeft) * 2 * PI / ticksPer2PI; // 计算当前角度,通过标定获得ticksPer2PI
        ROS_INFO("-------------dis------------%.10f,----------theta---------%.2f", detdistance, detth);

        linearSpeed = detdistance / velDeltaTime;
        angularSpeed = detth / velDeltaTime;

        // if (detdistance != 0)
        if (1)
        {
            x += detdistance * cos(th); // x坐标
            y += detdistance * sin(th); // y坐标
        }
        if (detth != 0)
        {
            th += detth; // 总角度
        }
        if (calibrate_lineSpeed == 1)
        {
            printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %d\n", x, y, th, linearSpeed, detEncode, LeftticksPerMeter, rightticksPerMeter, batteryVoltage);
        }

        // send command to stm32
        if (odomreset.data == 0){
            x = 0;
            y = 0;
            th = -1.57;
        }

        sendCarInfoKernel();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        nav_msgs::Odometry odom; // 创建nav_msgs::Odometry类型的消息odom
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // set the position
        // ROS_INFO("-------------------------%.2f,-------------------%.2f", &x, &y);
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = linearSpeed; // 线速度
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = angularSpeed; // 角速度

        // tfs.child_frame_id = "base_link";
        // tfs.header.frame_id = "odom";
        // tfs.header.stamp = ros::Time::now();
        // tfs.transform.translation.x = x;
        // tfs.transform.translation.y = y;
        // tfs.transform.translation.z = 0.0;
        // tf2::Quaternion qtn;
        // qtn.setRPY(0, 0, th);
        // tfs.transform.rotation.x = qtn.getX();
        // tfs.transform.rotation.y = qtn.getY();
        // tfs.transform.rotation.z = qtn.getZ();
        // tfs.transform.rotation.w = qtn.getW();
        // broadcaster.sendTransform(tfs);

        if (encoderLeft == 0 && encoderRight == 0)
        {
            odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9};

            odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                     0, 1e-3, 1e-9, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e-9};
        }
        else
        {
            odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};

            odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                     0, 1e-3, 0, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e3};
        }
        pub_odom.publish(odom);

#endif
        rate.sleep();
    }
}

//  运控发布到下位机stm32
void actuator::sendCarInfoKernel()
{
    // linear speed
    if (moveBaseControl.TargetSpeed > 0)
        moveBaseControl.TargetShiftPosition = 0x02; // 前进
    else if (moveBaseControl.TargetSpeed < 0)
        moveBaseControl.TargetShiftPosition = 0x01; // 后退
    else if (moveBaseControl.TargetSpeed == 0)
        moveBaseControl.TargetShiftPosition = 0x00; // 停止

    // 限制转动角度，防止进入死点
    if (moveBaseControl.TargetAngle < -120)
    {
        moveBaseControl.TargetAngle = -120; // 角度小于0时 等于0
    }
    if (moveBaseControl.TargetAngle > 120)
    {
        moveBaseControl.TargetAngle = 120; // 角度最大120
    }

    // 期望转向角度符号
    if (moveBaseControl.TargetAngle > 0)
        moveBaseControl.TargetAngleDir = 0x20; // 右转（左右跟舵机安装方式有关）
    else if (moveBaseControl.TargetAngle < 0)
        moveBaseControl.TargetAngleDir = 0x10; // 左转
    else if (moveBaseControl.TargetAngle == 0)
    {
        moveBaseControl.TargetAngleDir = 0x00; // 直行
    }

    // 起始位
    unsigned char buf[23] = {0};
    buf[0] = 0xa5;
    buf[1] = 0x5a;
    buf[2] = 0x06;
    // 保持和下位机一致的
    // printf("moveBaseControl.TargetAngle---------%.2f", &moveBaseControl.TargetAngle);
    buf[3] = (int)moveBaseControl.TargetAngleDir;      // targetangleDirection 0-->go straight,0x10-->turn left,0x20-->turn right (not used)
    buf[4] = (int)abs(moveBaseControl.TargetAngle);    // targetangle
    buf[5] = (int)abs(moveBaseControl.TargetSpeed);    // targetSpeed
    buf[6] = (int)moveBaseControl.TargetModeSelect;    // 0-->person control,1-->auto control (not used)
    buf[7] = (int)moveBaseControl.TargetShiftPosition; // targetshiftposition  0-->P stop;1-->R;2-->D. (not used)

    buf[8] = 8; // heart beat
    unsigned char sum = 0;
    for (int i = 2; i < 19; ++i)
        sum += buf[i];
    buf[9] = (unsigned char)(sum);
    size_t writesize = ser.write(buf, 10);
}
// 发布IMU数据
void actuator::pub_9250()
{
    sensor_msgs::Imu imuMsg;
    sensor_msgs::MagneticField magMsg;

    ros::Time current_time = ros::Time::now();

    imuMsg.header.stamp = current_time;
    imuMsg.header.frame_id = "imu_link";
    imuMsg.angular_velocity.x = gyroX;
    imuMsg.angular_velocity.y = gyroY;
    imuMsg.angular_velocity.z = gyroZ;
    imuMsg.angular_velocity_covariance = {
        0.04, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.04};

    imuMsg.linear_acceleration.x = accelX;
    imuMsg.linear_acceleration.y = accelY;
    imuMsg.linear_acceleration.z = accelZ;
    imuMsg.linear_acceleration_covariance = {
        0.04, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.04};
    pub_imu.publish(imuMsg); // 发布imuMsg

    magMsg.header.stamp = current_time;
    magMsg.header.frame_id = "base_link";
    magMsg.magnetic_field.x = magX;
    magMsg.magnetic_field.y = magY;
    magMsg.magnetic_field.z = magZ;
    magMsg.magnetic_field_covariance = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0};
    pub_mag.publish(magMsg); // 发布magMsg
}

// 接收下位机发送来的数据
void actuator::recvCarInfoKernel()
{
    std::string recvstr;
    unsigned char tempdata, lenrecv;
    unsigned char count, last_data, last_last_data, last_last_last_data;
    unsigned char str[100];
    bool recvflag = false;
    bool recvd_flag = false;
    memset(&str, 0, sizeof(str));
    ros::Time begin_time = ros::Time::now();
    double clustering_time = 0;

    while (1)
    {
        clustering_time = (ros::Time::now() - begin_time).toSec(); // 计算时间差，转换成秒
        if (clustering_time > 1)
        {
            recvd_flag = false;
            break;
        }

        recvstr = ser.read(1);
        if ((int)recvstr.size() != 1)
            continue;

        tempdata = recvstr[0];
        if (last_last_last_data == 0xa5 && last_last_data == 0x5a)
        {
            lenrecv = last_data;
            recvflag = true;
            count = 0;
        }
        if (recvflag)
        {
            str[count] = tempdata;
            count++;
            if (count == lenrecv)
            {
                recvflag = false;
                recvd_flag = true;
                break;
            }
        }
        last_last_last_data = last_last_data;
        last_last_data = last_data;
        last_data = tempdata;
    }

    if (recvd_flag)
    { // 数据解析，接收到的数据转存
        memcpy(&encoderLeft, str, 4);
        memcpy(&encoderRight, str + 4, 4);
        // ROS_INFO("------1----1-----1----------%d,------1----1-----1----%d", encoderLeft, encoderRight);

        memcpy(&batteryVoltage, str + 8, 4);
        // encoderRight = -encoderRight;
        // ROS_INFO("------2----2-----2----------%d,------2----2-----2----%d", encoderLeft, encoderRight);

        memcpy(&tempaccelX, str + 12, 2);
        memcpy(&tempaccelY, str + 14, 2);
        memcpy(&tempaccelZ, str + 16, 2);

        memcpy(&tempgyroX, str + 18, 2);
        memcpy(&tempgyroY, str + 20, 2);
        memcpy(&tempgyroZ, str + 22, 2);

        memcpy(&tempmagX, str + 24, 2);
        memcpy(&tempmagY, str + 26, 2);
        accelX = (float)tempaccelX / 2048 * 9.8; // 线加速度处理
        accelY = (float)tempaccelY / 2048 * 9.8;
        accelZ = (float)tempaccelZ / 2048 * 9.8;

        gyroX = (float)tempgyroX / 16.4 / 57.3; // 角速度处理
        gyroY = (float)tempgyroY / 16.4 / 57.3;
        gyroZ = (float)tempgyroZ / 16.4 / 57.3;

        magX = (float)tempmagX * 0.14; // 磁力计处理
        magY = (float)tempmagY * 0.14;
        magZ = (float)tempmagZ * 0.14;

        if (encoderLeft > 220 || encoderLeft < -220)
        {
            encoderLeft = 0;
            ROS_INFO(">220-LEFT--wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww");
        };
        if (encoderRight > 220 || encoderRight < -220)
        {
            encoderRight = 0;
            ROS_INFO(">220-RIGHT--qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq");
        };
        LeftticksPerMeter += encoderLeft;   // 获得左轮总脉冲数
        rightticksPerMeter += encoderRight; // 获得右轮总脉冲数
    }
}