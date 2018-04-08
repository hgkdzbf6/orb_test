#include "sift/lib/orb_main.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
OrbMain::OrbMain():
mats_(UAV_NUM),
keypoints_(UAV_NUM),
descriptors_(UAV_NUM),
match_pubs_(UAV_NUM),
good_matches_(UAV_NUM),
my_id_(0),
init_ok_(false)
{
  
}

void OrbMain::run(){
  
}
void OrbMain::singleMatch(int index){
  // 自身的编号和接受到的图像的信号不一样。
  // ROS_ASSERT(index!=my_id_);
  if(is_same_){ 
    pose_.header.stamp=ros::Time::now();
    pose_.pose.position.x=0.01;
    pose_.pose.position.y=0.01;
    pose_.pose.position.z=0;
    pose_.pose.orientation.w=1;
    pose_.pose.orientation.x=0;
    pose_.pose.orientation.y=0;
    pose_.pose.orientation.z=0;
    return;
  }
  try  
  { 
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_[0], descriptors_[1], matches );
    
    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_[0].rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::DMatch > good_matches;
    for ( int i = 0; i <  descriptors_[0].rows; i++ )
    {
        if ( matches[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    cv::Mat img_match; 
    cv::Mat img_goodmatch;
    cv::drawMatches ( mats_[0], keypoints_[0], mats_[1], keypoints_[1], matches, img_match );
    cv::drawMatches ( mats_[0], keypoints_[0], mats_[1], keypoints_[1], good_matches, img_goodmatch );
    good_matches_[0]=img_goodmatch;
    // ROS_INFO("%d",img_goodmatch.rows);
    // match_pubs_[0].publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_goodmatch).toImageMsg());
    // cv::imshow ( "所有匹配点对", img_match );
    // cv::imshow ( "优化后匹配点对", img_goodmatch );
    cv::Mat R,t;
    pose_estimation_2d2d(keypoints_[0],keypoints_[1],good_matches,R,t);
    //这边用了个trick,默认朝向的旋转不大
    // 把自己套进去了= 0
    // 查过这里的R矩阵，类型是双精度浮点数 
    Eigen::Matrix3d the_R;
    Eigen::Vector3d the_t;
    cv::cv2eigen(R,the_R);
    cv::cv2eigen(t,the_t);
    Eigen::Quaterniond the_q=Eigen::Quaterniond(the_R);
    //ROS_INFO_STREAM("the_q:"<< the_q.getW() << ","<< the_q.getAxis().getX() << ","<< the_q.getAxis().getY()<< ","<<the_q.getAxis().getZ() );
    if((R.at<double>(0,0)>0.9 && fabs(t.at<double>(2,0))<1)){
      // ROS_INFO_STREAM(std::endl<<R<<std::endl<<t<<std::endl);.inverse()
      geometry_msgs::PoseStamped ps;
      ps.header.stamp=ros::Time::now();
      ps.pose.position.x=-the_t(1)/70.0;
      ps.pose.position.y=-the_t(0)/70.0;
      ps.pose.position.z=-the_t(2)/70.0;
      ps.pose.orientation.w=the_q.w();
      ps.pose.orientation.x=the_q.x();
      ps.pose.orientation.y=the_q.y();
      ps.pose.orientation.z=the_q.z();
      pose_=ps;
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

OrbMain::~OrbMain()
{

}

bool OrbMain::pose_estimation_2d2d ( std::vector<cv::KeyPoint> keypoints_1,
                            std::vector<cv::KeyPoint> keypoints_2,
                            std::vector< cv::DMatch > matches,
                            cv::Mat& R, cv::Mat& t )
{

  if(!init_ok_)return false;
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
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point,
     cv::RANSAC, 0.9999, 1);
    // cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography ( points1, points2, cv::RANSAC, 3 );
    // cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    cv::recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );    
    the_s_=findMatchAverageDistance(points1,points2);
    t=the_s_*t;
    // ROS_INFO_STREAM(std::endl<<R<<std::endl<<t<<std::endl);
    return true;
    // cout<<"R is "<<endl<<R<<endl;
    // cout<<"t is "<<endl<<t<<endl;
  } 
  catch(cv::Exception& e2){
    ROS_ERROR("opencv exception is %s", e2.what());  
    return false;  
  }
}

double OrbMain::findMatchAverageDistance(
    std::vector<cv::Point2f>& points1,
    std::vector<cv::Point2f>& points2){
    double sum_x=0.0;
    double sum_y=0.0;
    uint len=points1.size();
    CV_Assert(len==points2.size());
    uint i;
    for(i=0;i<points1.size();i++){
      sum_x+=(points2[i].x-points1[i].x);
      sum_y+=(points2[i].y-points1[i].y);
    }
    sum_x=sum_x/len;
    sum_y=sum_y/len;
    return sqrt(sum_x*sum_x+sum_y*sum_y);
}

void OrbMain::ExtractKeypoints(cv::Mat mat,int index){
  if(is_same_)return;
  try{
    //-- 第一步:检测 Oriented FAST 角点位置
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    mats_[index]=mat.clone();
    Ptr<ORB> detector = cv::ORB::create(500,1.6f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
    detector->detectAndCompute(mats_[index],Mat(),
        keypoints_[index],descriptors_[index]);
    cv::drawKeypoints( mats_[index], keypoints_[index], mats_[index], cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  }catch(cv::Exception& e2){
    ROS_ERROR("opencv exception is %s", e2.what());  
    return;  
  }
}

cv::Mat OrbMain::mat(int index){
  return mats_[index]; 
}

cv::Mat OrbMain::good_match(int index){
  return good_matches_[index];
}