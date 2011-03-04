#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>
#include "pcl/filters/extract_indices.h"
#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "pcl/kdtree/kdtree_flann.h"
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
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

float sqrG(float y)
{
    return y*y;
}
float distanceG(pcl::PointXYZRGBNormal p1,pcl::PointXYZRGBNormal p2)
{
    float ans=sqrG(p1.x-p2.x)+sqrG(p1.y-p2.y)+sqrG(p1.z-p2.z);//+sqrG(p1.normal_x-p2.normal_x);
    ans=sqrt(ans);
    return ans;

}
/* ---[ */
int
  main (int argc, char** argv)
{
    float radius=0.005;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;    
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB> ()),cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

int stage=1;
  // Fill in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  if(stage<=1)
{
  reader.read<pcl::PointXYZRGB> ("/home/aa755/combined.pcd", *cloud);

    pcl::PassThrough<Point> pass_;
 pass_.setInputCloud (cloud);
  pass_.filter (*cloud_filtered);

  // Fill in the cloud data

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered1);

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  ror.setInputCloud (cloud_filtered1);
  std::cerr << "before radius : " << cloud_filtered1->size()<<std::endl;
  ror.setRadiusSearch(0.999*radius);
  ror.setMinNeighborsInRadius(2);
  ror.filter (*cloud_filtered2);
  std::cerr << "after radius : " <<cloud_filtered2->size()<<std::endl;

  writer.write<pcl::PointXYZRGB> ("inliers.pcd", *cloud_filtered2, false);
}
else if(stage==2)
{
  reader.read<pcl::PointXYZRGB> ("inliers.pcd", *cloud_filtered2);
}
  // release sor.
//  sor.setNegative (true);
//  sor.filter (*cloud_filtered);
//  writer.write<pcl::PointXYZRGB> ("outliers.pcd", *cloud_filtered, false);
if(stage<=2)
{
   pcl::NormalEstimation<Point, pcl::Normal> n3d_;
     KdTreePtr normals_tree_, clusters_tree_;
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
//  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
pcl::PointCloud<pcl::Normal> cloud_normals;
  // Normal estimation parameters
  n3d_.setKSearch (10);  
  n3d_.setSearchMethod (normals_tree_);
   n3d_.setInputCloud (cloud_filtered2);
  n3d_.compute (cloud_normals); 
  writer.write<pcl::Normal> ("normals.pcd", cloud_normals, false);
 pcl::concatenateFields (*cloud_filtered2, cloud_normals, *final_cloud);
  writer.write<pcl::PointXYZRGBNormal> ("xyzRGBnormals.pcd", *final_cloud, false);
}
else
  reader.read<pcl::PointXYZRGBNormal> ("xyzRGBnormals.pcd", *final_cloud);
  
  size_t numPoints=final_cloud->size();
  std::cerr << "number of points : " << numPoints<<std::endl;
  std::vector<int> k_indices;
  std::vector<float> k_distances;

  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> nnFinder;
  nnFinder.setInputCloud(final_cloud);


  ofstream myfile;
  myfile.open ("numNeighbors.005.txt");
  size_t numNeighbors;
//  delete cloud;
//  delete cloud_filtered;
//  delete cloud_filtered2;
//  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
   //pcl::ExtractIndices<pcl::PointXYZ> extract;
//   vector<int> isolated
  for(size_t i=0;i<numPoints;i++)
  {
    numNeighbors=nnFinder.radiusSearch(i,radius,k_indices,k_distances,20);
    myfile<<numNeighbors<<endl;
  }
  
  myfile.close();

  return (0);
}
/* ]--- */
