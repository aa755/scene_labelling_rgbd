

#include <stdint.h>

#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "segmentAndLabel.h"

typedef pcl::PointXYZRGBCamSL PointT;


typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;








/* ---[ */
int
  main (int argc, char** argv)
{


  sensor_msgs::PointCloud2 cloud_blob1, cloud_blob2;

  pcl::PointCloud<PointT> cloud;

  pcl::PCDWriter writer;

 // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob1) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s  with the following fields: %s", (int)(cloud_blob1.width * cloud_blob1.height), argv[1], pcl::getFieldsList (cloud_blob1).c_str ());

  if (pcl::io::loadPCDFile (argv[2], cloud_blob2) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s  with the following fields: %s", (int)(cloud_blob2.width * cloud_blob2.height), argv[2], pcl::getFieldsList (cloud_blob2).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blob1, cloud);
  
   pcl::PointCloud<PointFrameT> cloud_frame;
   pcl::fromROSMsg (cloud_blob2, cloud_frame);
   
   pcl::PointCloud<PointT> cloud_frame_labeled;
   findlabel (cloud, cloud_frame, cloud_frame_labeled);

   pcl::PointCloud<PointT> cloud_frame_segmented,cloud_frame_seg_labeled;
   segment (cloud_frame_labeled, cloud_frame_segmented);
   findConsistentLabels (cloud_frame_segmented,cloud_frame_seg_labeled);
   



 
  // extractEuclideanClusters ( *cloud_filtered, *cloud_normals_ptr, clusters_tree_, radius, hue_tolerance, clusters,angle, min_pts_per_cluster, max_pts_per_cluster);
  

  std::string fn (argv[2]);
  fn = fn.substr(0,fn.find('.'));

  fn = fn + "_labeled.pcd";
  writer.write ( fn,cloud_frame_labeled, true);

cout <<"wrote file "<<fn<<endl;
  return (0);
}
/* ]--- */
