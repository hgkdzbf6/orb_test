#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include "sift/sift_main.hpp"
#include <std_srvs/Trigger.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sift");
    ros::NodeHandle nh;
    SiftMain sift_main;
    ros::spin();
    // 这边是图像的订阅器
    // image_transport::ImageTransport it(nh);    
    return 0;
}
