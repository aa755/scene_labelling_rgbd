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
#include "feat_utils.h"
#include "segmentAndLabel.h"

using namespace pcl;



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

  
   if(++callback_counter_%step_ == 0) {
   ROS_INFO("Received data from kinect");
   VectorG origin(0,0,0);
       pcl::PointCloud<pcl::PointXYZRGB> cloud;
       pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
       pcl::fromROSMsg(*point_cloud, cloud);
       convertType(cloud,*cloud_seg_ptr,origin,0);
       segmentInPlace(*cloud_seg_ptr);
   }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"hi");
  unsigned int step = 10;
  if(argc > 1)  step = atoi(argv[1]);
  ros::NodeHandle n;
  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(n, 
                                 "/camera/rgb/image_mono",  
                                 "/camera/depth/image",
								 "/camera/rgb/camera_info",
								 "/camera/rgb/points");
  
 
   ros::spin();
  

}


