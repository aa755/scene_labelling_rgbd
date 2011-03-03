#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>


/* ---[ */
int
  main (int argc, char** argv)
{
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;    
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB> ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB> ("/home/aa755/combined.pcd", *cloud);


  // Fill in the cloud data

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("inliers.pcd", *cloud_filtered, false);
    pcl::PassThrough<Point> pass_;
 pass_.setInputCloud (cloud_filtered);
  pass_.filter (*cloud_filtered2);
  // release sor.
//  sor.setNegative (true);
//  sor.filter (*cloud_filtered);
//  writer.write<pcl::PointXYZRGB> ("outliers.pcd", *cloud_filtered, false);

   pcl::NormalEstimation<Point, pcl::Normal> n3d_;
     KdTreePtr normals_tree_, clusters_tree_;
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
pcl::PointCloud<pcl::Normal> cloud_normals;
  // Normal estimation parameters
  n3d_.setKSearch (10);  
  n3d_.setSearchMethod (normals_tree_);
   n3d_.setInputCloud (cloud_filtered2);
  n3d_.compute (cloud_normals); 
  writer.write<pcl::Normal> ("normals.pcd", cloud_normals, false);
  return (0);
}
/* ]--- */
