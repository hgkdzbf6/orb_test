#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "image_transport/image_transport.h"  
#include "cv_bridge/cv_bridge.h"  
#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/bind.hpp>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const int UAV_NUM = 2;

//这里想做模板匹配
class SiftMain
{
public:
	SiftMain();
	virtual ~SiftMain();
	void run();
private:
	bool init_ok_;
	int uav_index_;
	ros::NodeHandle nh_;
	// 算了为了可扩展性,直接上数组吧
	// 这个应该一个就行, 不用数组了
	// std::vector<image_transport::ImageTransport> its_(UAV_NUM);
	image_transport::ImageTransport it_;
	ros::ServiceServer ss_;
	
	std::vector<ros::Publisher> t_pubs_;

	std::vector<image_transport::Subscriber> image_subs_;
	std::vector<image_transport::Publisher> image_pubs_;
	
	std::vector<std::string> sub_strs_;
	std::vector<std::string> pub_strs_;

	std::vector<cv::Mat> mats_;
	std::vector<std::vector<cv::KeyPoint> > keypoints_;
	std::vector<cv::Mat> descriptors_;

	std::vector<image_transport::Publisher> match_pubs_;
	ros::Timer timer_;
	// 问题: 如何传入更多的参数? 不然一个数组的话就不知道谁是谁了
	// 参考: https://www.cnblogs.com/TIANHUAHUA/p/8418818.html
	void ImageCb(const sensor_msgs::ImageConstPtr& msg ,const int& index);
	void timerCallback(const ros::TimerEvent&);
	bool pose_estimation_2d2d( std::vector<cv::KeyPoint> keypoints_1,
                            std::vector<cv::KeyPoint> keypoints_2,
                            std::vector< cv::DMatch > matches,
                            cv::Mat& R, cv::Mat& t );

	bool callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
	void singleMatch(int i);
};