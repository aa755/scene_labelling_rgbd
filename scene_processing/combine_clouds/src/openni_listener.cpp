//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/CvBridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cv.h>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


boost::numeric::ublas::matrix<double> transformAsMatrix(const tf::Transform& bt)
{
   boost::numeric::ublas::matrix<double> outMat(4,4);

   //  double * mat = outMat.Store();

   double mv[12];
   bt.getBasis().getOpenGLSubMatrix(mv);

   btVector3 origin = bt.getOrigin();

   outMat(0,0)= mv[0];
   outMat(0,1)  = mv[4];
   outMat(0,2)  = mv[8];
   outMat(1,0)  = mv[1];
   outMat(1,1)  = mv[5];
   outMat(1,2)  = mv[9];
   outMat(2,0)  = mv[2];
   outMat(2,1)  = mv[6];
   outMat(2,2) = mv[10];

   outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
   outMat(0,3) = origin.x();
   outMat(1,3) = origin.y();
   outMat(2,3) = origin.z();
   outMat(3,3) = 1;


   return outMat;
}

using namespace cv;



void transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,
                          sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32) {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }
  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");
  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i) {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;
    
    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2])) {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr)) { // Invalid point 
        pt_out = pt;
      } else { // max range point
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    } else {
      pt_out = transform * pt;
    }

    if (max_range_point) {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
      //std::cout << __PRETTY_FUNCTION__<<": "<<i << "is a max range point.\n";
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
    
    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1) {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i) {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

//Little debugging helper functions
std::string openCVCode2String(unsigned int code){
  switch(code){
    case 0 : return std::string("CV_8UC1" );
    case 8 : return std::string("CV_8UC2" );
    case 16: return std::string("CV_8UC3" );
    case 24: return std::string("CV_8UC4" );
    case 2 : return std::string("CV_16UC1");
    case 10: return std::string("CV_16UC2");
    case 18: return std::string("CV_16UC3");
    case 26: return std::string("CV_16UC4");
    case 5 : return std::string("CV_32FC1");
    case 13: return std::string("CV_32FC2");
    case 21: return std::string("CV_32FC3");
    case 29: return std::string("CV_32FC4");
  }
  return std::string("Unknown");
}

void printMatrixInfo(cv::Mat& image){
  ROS_DEBUG_STREAM("Matrix Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}

OpenNIListener::OpenNIListener( ros::NodeHandle nh, const char* pointcloud_topic, const char* t_pointcloud_topic )
: pointcloud_sub_ (nh, pointcloud_topic, 3),
  t_pointcloud_sub_(nh, t_pointcloud_topic, 3),
  sync_(MySyncPolicy(10),  pointcloud_sub_ , t_pointcloud_sub_),
  cloudMerged (new pcl::PointCloud<pcl::PointXYZRGB> ()),
  callback_counter_(0)
{
    firstFrame=true;
    count=0;
    std::cerr<<"first frame false"<<endl;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.registerCallback(boost::bind(&OpenNIListener::cameraCallback, this, _1, _2 ));
  ROS_INFO_STREAM("OpenNIListener listening to " << pointcloud_topic << ", " << t_pointcloud_topic << "\n");
  sub_ = nh.subscribe(pointcloud_topic, 1000, &OpenNIListener::Callback, this);

//  matcher_ = new cv::BruteForceMatcher<cv::L2<float> >() ;
//  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
//            "/rgbdslam/cloud", 5);
//  pub_transf_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
//            "/rgbdslam/transformed_slowdown_cloud", 5);
//  pub_ref_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
 //           "/rgbdslam/first_frame", 1);
}

void OpenNIListener::Callback (const sensor_msgs::PointCloud2ConstPtr&  point_cloud)
{
   ROS_INFO ("Got the msg");
}



void OpenNIListener::cameraCallback (const sensor_msgs::PointCloud2ConstPtr&  point_cloud,
                                     const sensor_msgs::PointCloud2ConstPtr& t_point_cloud) {
   ROS_INFO("Received data from kinect");
   tf::StampedTransform transform;
   try{
     listener.lookupTransform("/openni_camera", "/batch_transform",
                              point_cloud->header.stamp, transform);
     ROS_INFO ("origin: %f %f %f", transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
     ROS_INFO ("rotation: %f %f %f %f", transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ(),transform.getRotation().getW());
     TransformG transfG(transform);
     transfG.print();

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZRGB> ());

         applyFilters(t_point_cloud,cloudTemp);
         std::stringstream ss;
    ss << count;

        writer.write<pcl::PointXYZRGB> ("/home/aa755/tempv/pc"+ss.str()+".pcd", *cloudTemp, false);

         count++;
     if(firstFrame)
     {
         firstFrame=false;
           *cloudMerged=*cloudTemp;
        //writer.write<pcl::PointXYZRGB> ("/home/aa755/VisibilityMerged.pcd", *cloudMerged, false);
        std::cerr<<"wrote  pcl countaining "<< cloudMerged->size()<<endl;
        
     }
     else if(count%5==2)
     {
         *cloudMerged+=*cloudTemp;
         if(count>20)
        writer.write<pcl::PointXYZRGB> ("/home/aa755/VisibilityMerged.pcd", *cloudMerged, false);
        std::cerr<<"wrote  pcl countaining "<< cloudMerged->size()<<" "<<count<<endl;
         
     }
     
         
   }
   catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
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
