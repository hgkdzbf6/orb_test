#include <iostream>
#include "image_transport/image_transport.h"  
#include "cv_bridge/cv_bridge.h"  
#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/bind.hpp>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include "sift/lib/orb_main.hpp"


//这里想做模板匹配
class OrbNode
{
public:
	OrbNode();
	virtual ~OrbNode();
	void run();
private:
	OrbMain orb_main;
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

	std::vector<image_transport::Publisher> match_pubs_;
	ros::Timer timer_;
	// 问题: 如何传入更多的参数? 不然一个数组的话就不知道谁是谁了
	// 参考: https://www.cnblogs.com/TIANHUAHUA/p/8418818.html
	void ImageCb(const sensor_msgs::ImageConstPtr& msg ,const int& index);
	void timerCallback(const ros::TimerEvent&);
	bool callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

};