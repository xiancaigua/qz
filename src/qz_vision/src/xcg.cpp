#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

image_transport::Publisher pub_image;
cv_bridge::CvImagePtr cv_ptr;

// 定义回调函数
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("read camera success ......");
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        if (img.empty())
        {
            ROS_INFO("img is empty");
        }
        sensor_msgs::ImagePtr img2msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub_image.publish(img2msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    // 初始化节点，定义节点名字
    ros::init(argc, argv, "image_sub_pub_node");
    ros::NodeHandle n;

    // 创建一个发布者(publisher)发布名为camera的话题(topic),消息队列长度为１
    image_transport::ImageTransport it1(n);
    pub_image = it1.advertise("/XCG", 10);

    // 创建窗口用于显示图像
    ROS_INFO("camera info ------------------ ");
    // 创建一个订阅者，订阅名为camera的话题(topic),　注册回调函数
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/Img", 10, imageCallback);
    // 循环等待回调函数
    ROS_INFO("camera info ------------------ ");
    ros::spin();
}