#include "float.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"


//typedef pcl::PointXYGRGBCam PointT;
typedef pcl::PointXYZRGBCamSL PointT;

using namespace pcl;

void getMinMax ( const pcl::PointCloud<PointT> &cloud, const IndicesPtr &indices, Eigen::Vector4f &min_p, Eigen::Vector4f &max_p)
{
   min_p.setConstant (FLT_MAX);
   max_p.setConstant (-FLT_MAX);
   min_p[3] = max_p[3] = 0;
 
   for (size_t i = 0; i < indices->size (); ++i)
   {
     if (cloud.points[(*indices)[i]].x < min_p[0]) min_p[0] = cloud.points[(*indices)[i]].x;
     if (cloud.points[(*indices)[i]].y < min_p[1]) min_p[1] = cloud.points[(*indices)[i]].y;
     if (cloud.points[(*indices)[i]].z < min_p[2]) min_p[2] = cloud.points[(*indices)[i]].z;
 
     if (cloud.points[(*indices)[i]].x > max_p[0]) max_p[0] = cloud.points[(*indices)[i]].x;
     if (cloud.points[(*indices)[i]].y > max_p[1]) max_p[1] = cloud.points[(*indices)[i]].y;
     if (cloud.points[(*indices)[i]].z > max_p[2]) max_p[2] = cloud.points[(*indices)[i]].z;
   }
}


void getMinMax ( const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &min_p, Eigen::Vector4f &max_p)
{
   min_p.setConstant (FLT_MAX);
   max_p.setConstant (-FLT_MAX);
   min_p[3] = max_p[3] = 0;
 
   for (size_t i = 0; i < cloud.points.size (); ++i)
   {
     if (cloud.points[i].x < min_p[0]) min_p[0] = cloud.points[i].x;
     if (cloud.points[i].y < min_p[1]) min_p[1] = cloud.points[i].y;
     if (cloud.points[i].z < min_p[2]) min_p[2] = cloud.points[i].z;
 
     if (cloud.points[i].x > max_p[0]) max_p[0] = cloud.points[i].x;
     if (cloud.points[i].y > max_p[1]) max_p[1] = cloud.points[i].y;
     if (cloud.points[i].z > max_p[2]) max_p[2] = cloud.points[i].z;
   }
}




int main (int argc, char** argv)
{

  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<PointT> cloud;
  
  // read the pcd file 

  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());
  
  // convert to templated message type

   pcl::fromROSMsg (cloud_blob, cloud);
   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
   pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
   pcl::PointIndices::Ptr segment_indices (new pcl::PointIndices ());

  // get segments 

  // find the max segment number 
   int max_segment_num = 0;
   for (size_t i = 0; i < cloud.points.size (); ++i)
   {
     if (max_segment_num < cloud.points[i].segment) {max_segment_num = cloud.points[i].segment;}
   }

   PassThrough<PointT> pass_segment;
   pass_segment.setInputCloud (cloud_ptr);
   pass_segment.setFilterFieldName("segment");
   Eigen::Vector4f min_p;
   Eigen::Vector4f max_p;
   for (int i = 1 ; i <= max_segment_num ; i ++)
   {
      pass_segment.setFilterLimits(i,i);
      pass_segment.filter (*cloud_filtered);
      getMinMax(*cloud_filtered,min_p,max_p);
   }

  // for each segment compute features
  // feature 1 : height 


}


