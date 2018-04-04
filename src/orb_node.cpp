#include "sift/node/orb_node.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
OrbNode::OrbNode():
it_(nh_),
sub_strs_(UAV_NUM),
pub_strs_(UAV_NUM),
image_subs_(UAV_NUM),
image_pubs_(UAV_NUM),
match_pubs_(UAV_NUM*(UAV_NUM-1)*2),
t_pubs_(UAV_NUM*(UAV_NUM-1)*2)
{
  // image_pub_=it_.advertise("out",2);
  // image_sub_=it_.subscribe("/hummingbird/svo/image",2,&OrbNode::ImageCb,this);
  
	ros::NodeHandle pnh("~");
  const std::string slash="/";
  const std::string str="hummingbird";
  std::string sub_str;
  std::string pub_str;
  std::string sub_name_str;
  std::string pub_name_str;
  // 读入参数,这里用私有的命名空间
  pnh.param<int>("uav_index",orb_main.my_id_,0);
  pnh.param<int>("other_index",orb_main.other_id_,0);
  
  // 自身 
  sub_name_str=str+std::to_string(orb_main.my_id_)+"_sub_image";
  pub_name_str=str+std::to_string(orb_main.my_id_)+"_pub_image";
  sub_str=slash+str+std::to_string(orb_main.my_id_)+"/camera_nadir/image_raw";
  pub_str=slash+str+std::to_string(orb_main.my_id_)+"/out/image";
  pnh.param<std::string>(sub_name_str,sub_strs_[0],sub_str);
  pnh.param<std::string>(pub_name_str,pub_strs_[0],pub_str);

  // 另一个
  sub_name_str=str+std::to_string(orb_main.other_id_)+"_sub_image";
  pub_name_str=str+std::to_string(orb_main.other_id_)+"_pub_image";
  sub_str=slash+str+std::to_string(orb_main.other_id_)+"/camera_nadir/image_raw";
  pub_str=slash+str+std::to_string(orb_main.other_id_)+"/out/image";
  pnh.param<std::string>(sub_name_str,sub_strs_[1],sub_str);
  pnh.param<std::string>(pub_name_str,pub_strs_[1],pub_str);

  ss_= nh_.advertiseService("receive_image", &OrbNode::callback,this);  
  // 订阅和发布主题,用公有的命名空间
  // 单一函数
  // image_subs_[i]=it_.subscribe(sub_strs_[i],1,&OrbNode::ImageCb2,this);
  // 多个函数
  // 详细见 http://blog.csdn.net/sunfc_nbu/article/details/52881656
  image_subs_[0]=it_.subscribe(sub_strs_[0],1,boost::bind(&OrbNode::ImageCb,this,_1,0));
  image_pubs_[0]=it_.advertise(pub_strs_[0],1);    
  image_subs_[1]=it_.subscribe(sub_strs_[1],1,boost::bind(&OrbNode::ImageCb,this,_1,1));
  image_pubs_[1]=it_.advertise(pub_strs_[1],1);

  // std::cout <<"wtf????????????" << std::endl;
  // 这里定义匹配,按照穿过的图像进行匹配

  // 用浪费点空间的方法省点编写时间吧
  // 广播匹配图像,后期可以不用
  match_pubs_[0]=it_.advertise("match"+std::to_string(orb_main.my_id_)
    +std::to_string(orb_main.other_id_),1);
  // 广播位移向量(未来有可能加入旋转)
  t_pubs_[0]=nh_.advertise<geometry_msgs::PoseStamped>("relative_pose"+
    std::to_string(orb_main.my_id_)+std::to_string(orb_main.other_id_),5);

  // std::cout <<"wtf???????????????????????" << std::endl;
  // 定时器,0.1s
  timer_ = nh_.createTimer(ros::Duration(0.2), &OrbNode::timerCallback,this);
}

bool OrbNode::callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{  
  orb_main.init_ok_=true;
  response.message="get relative position start, from " + 
    std::to_string(orb_main.my_id_)+ "to " + std::to_string(orb_main.other_id_);
	response.success=true;
  return true;
}

void OrbNode::run(){
  
}
void OrbNode::timerCallback(const ros::TimerEvent&){
  if(!orb_main.init_ok_)return ;
  orb_main.singleMatch(0);
  match_pubs_[0].publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", orb_main.good_match(0)).toImageMsg());
  if(orb_main.pose_.pose.position.x==0 && orb_main.pose_.pose.position.y==0 )
    return ;  
  t_pubs_[0].publish(orb_main.pose_);
}

OrbNode::~OrbNode()
{

}

void OrbNode::ImageCb(const sensor_msgs::ImageConstPtr& msg,const int& index)  
{  
  if(!orb_main.init_ok_)return;
  cv_bridge::CvImagePtr cv_ptr;  
  cv::Mat mat;
  // ROS_INFO("hello ");
  try  
  {  
    /*转化成CVImage*/  
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
    // if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)  
    //   cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));  
    cv::cvtColor(cv_ptr->image, mat, CV_RGB2GRAY);
    orb_main.ExtractKeypoints(mat,index);
    image_pubs_[index].publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", orb_main.mat(index)).toImageMsg());  
  }  
  catch (cv_bridge::Exception& e)  
  {  
    ROS_ERROR("cv_bridge exception is %s", e.what());  
    return;  
  } catch(cv::Exception& e2){
    ROS_ERROR("opencv exception is %s", e2.what());  
    return;  
  }
}  

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb");
    ros::NodeHandle nh;
    OrbNode orb_main;
    ros::spin();
    // 这边是图像的订阅器
    // image_transport::ImageTransport it(nh);    
    return 0;
}