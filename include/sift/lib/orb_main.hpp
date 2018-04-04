#include <iostream>
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
#include <opencv2/core/eigen.hpp>

static const int UAV_NUM = 2;

//这里想做模板匹配
class OrbMain
{
public:
	OrbMain();
	virtual ~OrbMain();
	void run();
	// 是否初始化完成
	bool init_ok_;
	// 自身id
	int my_id_;
	// 别的id
	int other_id_;
	// 保存的某个中间结果
	double the_s_;
	// 广播的relative_pose
	geometry_msgs::PoseStamped pose_;
	// 算了为了可扩展性,直接上数组吧
	// 这个应该一个就行, 不用数组了
	// std::vector<image_transport::ImageTransport> its_(UAV_NUM);

	std::vector<cv::Mat> mats_;
	std::vector<std::vector<cv::KeyPoint> > keypoints_;
	std::vector<cv::Mat> descriptors_;
	std::vector<cv::Mat> good_matches_;
	std::vector<image_transport::Publisher> match_pubs_;
	// ros::Timer timer_;
	// 问题: 如何传入更多的参数? 不然一个数组的话就不知道谁是谁了
	// 参考: https://www.cnblogs.com/TIANHUAHUA/p/8418818.html

	bool pose_estimation_2d2d( std::vector<cv::KeyPoint> keypoints_1,
                            std::vector<cv::KeyPoint> keypoints_2,
                            std::vector< cv::DMatch > matches,
                            cv::Mat& R, cv::Mat& t );
	void singleMatch(int i);
	double findMatchAverageDistance(
		std::vector<cv::Point2f>& points1,
		std::vector<cv::Point2f>& points2);
	void ExtractKeypoints(cv::Mat mat,int index);
	cv::Mat mat(int index);
	cv::Mat good_match(int index);

};
