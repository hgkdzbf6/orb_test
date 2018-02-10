#include "sift/orb_main.hpp"

OrbMain::OrbMain():
it_(nh_),
sub_strs_(UAV_NUM),
pub_strs_(UAV_NUM),
image_subs_(UAV_NUM),
image_pubs_(UAV_NUM),
mats_(UAV_NUM),
keypoints_(UAV_NUM),
descriptors_(UAV_NUM),
match_pubs_(UAV_NUM),
t_pubs_(UAV_NUM),
uav_index_(0),
init_ok(false)
{
  // image_pub_=it_.advertise("out",2);
  // image_sub_=it_.subscribe("/hummingbird/svo/image",2,&OrbMain::ImageCb,this);
  
	ros::NodeHandle pnh("~");
  int i;
  const std::string slash="/";
  const std::string str="hummingbird";
  std::string sub_str;
  std::string pub_str;
  std::string sub_name_str;
  std::string pub_name_str;
  // 读入参数,这里用私有的命名空间
  pnh.param<int>("uav_index",uav_index_,0);
  for(i=0;i<UAV_NUM;i++){
    sub_name_str=str+std::to_string(i)+"_sub_image";
    pub_name_str=str+std::to_string(i)+"_pub_image";
    sub_str=slash+str+std::to_string(i)+"/camera_nadir/image_raw";
    pub_str=slash+str+std::to_string(i)+"/out/image";
    pnh.param<std::string>(sub_name_str,sub_strs_[i],sub_str);
    pnh.param<std::string>(pub_name_str,pub_strs_[i],pub_str);
  }
  ss_= nh_.advertiseService("taking_off2", &OrbMain::callback,this);  
  // 订阅和发布主题,用公有的命名空间
  for(i=0;i<UAV_NUM;i++){
    // 单一函数
    // image_subs_[i]=it_.subscribe(sub_strs_[i],1,&OrbMain::ImageCb2,this);
    // 多个函数
    // 详细见 http://blog.csdn.net/sunfc_nbu/article/details/52881656
    image_subs_[i]=it_.subscribe(sub_strs_[i],1,boost::bind(&OrbMain::ImageCb,this,_1,i));
    image_pubs_[i]=it_.advertise(pub_strs_[i],1);
  }  
  // std::cout <<"wtf????????????" << std::endl;
  // 这里定义匹配,按照穿过的图像进行匹配

  // 用浪费点空间的方法省点编写时间吧
  for(i=0;i<UAV_NUM;i++){
    if(i==uav_index_)continue;
    // 广播匹配图像,后期可以不用
    match_pubs_[i]=it_.advertise("pipei"+std::to_string(uav_index_)+std::to_string(i),1);
    // 广播位移向量(未来有可能加入旋转)
    t_pubs_[i]=nh_.advertise<geometry_msgs::PoseStamped>("relative_position"+std::to_string(uav_index_)+std::to_string(i),5);
  }

  // std::cout <<"wtf???????????????????????" << std::endl;
  // 定时器,0.1s
  timer_ = nh_.createTimer(ros::Duration(0.1), &OrbMain::timerCallback,this);
}

bool OrbMain::callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{  
  this->init_ok=true;
  response.message="get relative position start";
	response.success=true;
  return true;
}

void OrbMain::run(){
}
void OrbMain::singleMatch(int index){
  ROS_ASSERT(index!=uav_index_);
  try  
  { 
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_[uav_index_], descriptors_[index], matches );
    
    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_[uav_index_].rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
      

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::DMatch > good_matches;
    for ( int i = 0; i <  descriptors_[uav_index_].rows; i++ )
    {
        if ( matches[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    cv::Mat img_match; 
    cv::Mat img_goodmatch;
    cv::drawMatches ( mats_[uav_index_], keypoints_[uav_index_], mats_[index], keypoints_[index], matches, img_match );
    cv::drawMatches ( mats_[uav_index_], keypoints_[uav_index_], mats_[index], keypoints_[index], good_matches, img_goodmatch );
    // ROS_INFO("%d",img_goodmatch.rows);
    match_pubs_[index].publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_goodmatch).toImageMsg());
    // cv::imshow ( "所有匹配点对", img_match );
    // cv::imshow ( "优化后匹配点对", img_goodmatch );
    cv::Mat R,t;
    pose_estimation_2d2d(keypoints_[uav_index_],keypoints_[index],good_matches,R,t);
    //这边用了个trick,默认朝向的旋转不大
    if((R.at<double>(0,0)>0.9)){
      // ROS_INFO_STREAM(std::endl<<R<<std::endl<<t<<std::endl);
      geometry_msgs::PoseStamped ps;
      ps.header.stamp=ros::Time::now();
      ps.pose.position.x=t.at<double>(0,0)*4.2857142857142857;
      ps.pose.position.y=t.at<double>(1,0)*4.2857142857142857;
      ps.pose.position.z=t.at<double>(2,0)*4.2857142857142857;
      // 旋转矩阵转四元数过一段时间再写,看有无必要(肯定有必要)

      t_pubs_[index].publish(ps);
    }
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
void OrbMain::timerCallback(const ros::TimerEvent&){
  if(!init_ok)return ;
  int i;
  for(i=0;i<UAV_NUM;i++){
    if(i==uav_index_)continue;
    singleMatch(i);
  }
}

OrbMain::~OrbMain()
{

}

void OrbMain::ImageCb(const sensor_msgs::ImageConstPtr& msg,const int& index)  
{  
  if(!init_ok)return;
  cv_bridge::CvImagePtr cv_ptr;  
  // ROS_INFO("hello ");
  try  
  {  
    /*转化成CVImage*/  
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
      // if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)  
    //   cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));  

    cv::cvtColor(cv_ptr->image, mats_[index], CV_RGB2GRAY);
    
    // std::vector<std::vector<cv::KeyPoint> > keypoints_1, keypoints_2;
    // cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( mats_[index],keypoints_[index] );
    // detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( mats_[index], keypoints_[index], descriptors_[index] );
    
    // cv::drawKeypoints( mats_[index], keypoints_[index], mats_[index], cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::drawKeypoints( mats_[index], keypoints_[index], mats_[index], cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  
    image_pubs_[index].publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", mats_[index]).toImageMsg());  

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


bool OrbMain::pose_estimation_2d2d ( std::vector<cv::KeyPoint> keypoints_1,
                            std::vector<cv::KeyPoint> keypoints_2,
                            std::vector< cv::DMatch > matches,
                            cv::Mat& R, cv::Mat& t )
{

  if(!init_ok)return false;
  try
  {
    // 相机内参,TUM Freiburg2
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 205.47, 0, 320.5, 0, 205.47, 240.5, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat ( points1, points2, CV_FM_8POINT );
    // cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    cv::Point2d principal_point ( 160, 160 );	//相机光心, TUM dataset标定值
    double focal_length = 168;			//相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point );
    // cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography ( points1, points2, cv::RANSAC, 3 );
    // cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    cv::recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    
    return true;
    // cout<<"R is "<<endl<<R<<endl;
    // cout<<"t is "<<endl<<t<<endl;
  } 
  catch(cv::Exception& e2){
    ROS_ERROR("opencv exception is %s", e2.what());  
    return false;  
  }
    
}