#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include "sift/orb_main.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb");
    ros::NodeHandle nh;
    OrbMain orb_main;
    ros::spin();
    // 这边是图像的订阅器
    // image_transport::ImageTransport it(nh);    
    return 0;
}
