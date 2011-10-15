/* The purpose of this class is to listen to 
 * synchronized image pairs from the kinect
 * convert them to opencv, process them, convert
 * them to a qt image and send them to the mainwindow
 * defined in qtcv.h/.cpp
 */
#ifndef OPENNI_LISTENER_H
#define OPENNI_LISTENER_H
#include "ros/ros.h"
#include <rosbag/bag.h>
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <Eigen/Core>
//#include <tf/transform_listener.h>


namespace s = sensor_msgs;
namespace m = message_filters;

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef m::sync_policies::ApproximateTime<s::Image, s::Image, s::CameraInfo, s::PointCloud2> MySyncPolicy;

class OpenNIListener  {

  public:
    ///Constructor: The listener needs to know the topic for the optical and depth image
    ///Valid detector types: FAST, STAR, SIFT, SURF, MSER, GFTT
    ///Valid extractor types: SIFT, SURF
    ///As a detector SURF is better, FAST is faster, GFTT is fast but really bad in combination the SURF Extractor
    OpenNIListener(ros::NodeHandle nh, 
				   const char* visual_topic= "/visual_in",  
                   const char* depth_topic= "/depth_in",
                   const char* info_topic= "/cam_info_in", //Kinect:/camera/rgb/camera_info
                   const char* cloud_topic= "/cloud_in",
                   const char* filename= "test.bag",
				   unsigned int step = 10);

//    void Callback (const sensor_msgs::PointCloud2ConstPtr&  point_cloud);

    /** For each synchronized pair of opitcal/depth image messages
     *  convert them to cv::Mat, rescale the depth range, convert
     *  to QImage and emit newDepthImage and newGrayImage 
     */
    void cameraCallback (const s::ImageConstPtr& visual_img,  
                         const s::ImageConstPtr& depth_img, 
                         const s::CameraInfoConstPtr& cam_info,
                         const s::PointCloud2ConstPtr& point_cloud);
 
  protected:
    m::Subscriber<s::Image> visual_sub_ ;
    m::Subscriber<s::Image> depth_sub_;
    m::Subscriber<s::CameraInfo> info_sub_;
    m::Subscriber<s::PointCloud2> cloud_sub_;
    m::Synchronizer<MySyncPolicy> sync_;

    ros::Publisher pub_cloud_;
    ros::Publisher pub_transf_cloud_;
    ros::Publisher pub_ref_cloud_;
  //  tf::TransformListener  listener; 
   //ros::Publisher pc_pub; 
     unsigned int callback_counter_;
    unsigned int step_;
    //bool pause_;
    //bool first_frame_;
    rosbag::Bag bag_;


};

//QVector<float>* transformPointCloud (const Eigen::Matrix4f transform, const s::PointCloud2 &in);
//Copied from pcl_tf/transform.cpp
//void transformPointCloud (const Eigen::Matrix4f &transform, 
  //                        const sensor_msgs::PointCloud2 &in,
//                          sensor_msgs::PointCloud2 &out);


///Return the macro string for the cv::Mat type integer
//std::string openCVCode2String(unsigned int code);

///Print Type and size of image
//void printMatrixInfo(cv::Mat& image);
#endif
