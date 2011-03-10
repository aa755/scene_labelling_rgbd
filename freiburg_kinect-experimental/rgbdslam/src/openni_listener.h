/* The purpose of this class is to listen to 
 * synchronized image pairs from the kinect
 * convert them to opencv, process them, convert
 * them to a qt image and send them to the mainwindow
 * defined in qtcv.h/.cpp
 */
#ifndef OPENNI_LISTENER_H
#define OPENNI_LISTENER_H
#include "ros/ros.h"
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "graph_manager.h"
#include <QImage> //for cvMat2QImage not listet here but defined in cpp file
#include <QVector>//for transformPointCloud not listet here but defined in cpp file
#include <Eigen/Core>


namespace s = sensor_msgs;
namespace m = message_filters;

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef m::sync_policies::ApproximateTime<s::Image, s::Image, s::CameraInfo, s::PointCloud2> MySyncPolicy;

class OpenNIListener : public QObject {
  ///QT Stuff, to communicate with the gui
  Q_OBJECT
  signals:
    ///Connect to this signal to get up-to-date optical images from the listener
    void newVisualImage(QImage);
    ///Connect to this signal to get up-to-date depth images from the listener
    void newDepthImage(QImage);
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    void pauseStatus(bool is_paused);
  public slots:
    ///Ignore new incoming data
    void togglePause();

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
                   const char* detector_type = "SURF", 
                   const char* extractor_type = "SURF"); 

    /** For each synchronized pair of opitcal/depth image messages
     *  convert them to cv::Mat, rescale the depth range, convert
     *  to QImage and emit newDepthImage and newGrayImage 
     */
    void cameraCallback (const s::ImageConstPtr& visual_img,  
                         const s::ImageConstPtr& depth_img, 
                         const s::CameraInfoConstPtr& cam_info,
                         const s::PointCloud2ConstPtr& point_cloud);
  
    ///The GraphManager uses the Node objects to do the actual SLAM
    ///Public, s.t. the qt signals can be connected to by the holder of the OpenNIListener
    GraphManager graph_mgr;
  protected:
    /// Create a QImage from image. The QImage stores its data in rgba_buffer_ (which is overwritten each call)
    QImage cvMat2QImage(const cv::Mat& image); 

    /// Creates Feature Detector Objects accordingt to the type.
    /// Possible detectorTypes: FAST, STAR, SIFT, SURF, GFTT
    /// FAST and SURF are the self-adjusting versions (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html#DynamicAdaptedFeatureDetector)
    cv::FeatureDetector* createDetector( const string& detectorType );
    /// Create an object to extract features at keypoints. The Exctractor is passed to the Node constructor and must be the same for each node.
    cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );

    //Variables
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher > matcher_;
    m::Subscriber<s::Image> visual_sub_ ;
    m::Subscriber<s::Image> depth_sub_;
    m::Subscriber<s::CameraInfo> info_sub_;
    m::Subscriber<s::PointCloud2> cloud_sub_;
    m::Synchronizer<MySyncPolicy> sync_;
    cv::Mat depth_mono8_img_;
    cv::Mat rgba_buffer_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_transf_cloud_;
    ros::Publisher pub_ref_cloud_;
    //ros::Publisher pc_pub; 
    unsigned int callback_counter_;
    bool pause_;
    bool first_frame_;
};

//QVector<float>* transformPointCloud (const Eigen::Matrix4f transform, const s::PointCloud2 &in);
//Copied from pcl_tf/transform.cpp
void transformPointCloud (const Eigen::Matrix4f &transform, 
                          const sensor_msgs::PointCloud2 &in,
                          sensor_msgs::PointCloud2 &out);

///Convert the CV_32FC1 image to CV_8UC1 with a fixed scale factor
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);

///Return the macro string for the cv::Mat type integer
std::string openCVCode2String(unsigned int code);

///Print Type and size of image
void printMatrixInfo(cv::Mat& image);
#endif
