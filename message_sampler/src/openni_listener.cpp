//Documentation see header file
//#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
//#include "pcl/common/transform.h"
//#include "pcl_ros/transforms.h"
#include "openni_listener.h"
//#include <cv_bridge/CvBridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
//#include <cv.h>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>


OpenNIListener::OpenNIListener( ros::NodeHandle nh,  const char* visual_topic, 
                               const char* depth_topic, const char* info_topic, 
                               const char* cloud_topic, const char* filename , unsigned int step ) 
: visual_sub_ (nh, visual_topic, 10),
  depth_sub_(nh, depth_topic, 10),
  info_sub_(nh, info_topic, 10),
  cloud_sub_(nh, cloud_topic, 10),
  sync_(MySyncPolicy(10),  visual_sub_, depth_sub_, info_sub_, cloud_sub_),
  callback_counter_(0),
  step_(step)
{
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.registerCallback(boost::bind(&OpenNIListener::cameraCallback, this, _1, _2, _3, _4));
  ROS_INFO_STREAM("OpenNIListener listening to " << visual_topic << ", " << depth_topic \
                   << ", " << info_topic << " and " << cloud_topic << "\n"); 

  bag_.open(filename, rosbag::bagmode::Write);

//  matcher_ = new cv::BruteForceMatcher<cv::L2<float> >() ;
/*  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
            "/rgbdslam/input/points", 10);
  pub_info_ = nh.advertise<sensor_msgs::CameraInfo> (
            "/rgbdslam/input/camera_info", 10);
  pub_visual_ = nh.advertise<sensor_msgs::Image> (
           "/rgbdslam/input/image_mono", 10);
  pub_depth_ = nh.advertise<sensor_msgs::Image> (
           "/rgbdslam/input/depth_image", 10);
*/

}

void OpenNIListener::cameraCallback (const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

   ROS_INFO("Received a frame from kinect");
  
   if(++callback_counter_%step_ == 0) {
   ROS_INFO("accepted the frame");
     bag_.write ( "/camera/rgb/image_mono" , visual_img_msg->header.stamp, visual_img_msg );
     bag_.write ( "/camera/depth/image" , depth_img_msg->header.stamp, depth_img_msg );
     bag_.write ( "/camera/rgb/camera_info" , cam_info->header.stamp, cam_info );
     bag_.write ( "/camera/rgb/points" , point_cloud->header.stamp, point_cloud );
   }
/*  if(++callback_counter_ % 3 != 0 || pause_) return;
  //Get images into correct format
	sensor_msgs::CvBridge bridge;
	cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg); 
	cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg, "mono8");
  if(visual_img.rows != depth_float_img.rows || visual_img.cols != depth_float_img.cols){
    ROS_ERROR("Depth and Visual image differ in size!");
    return;
  }
   
*/

}

