#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sift");
    ros::NodeHandle nh;

    return 0;
}
