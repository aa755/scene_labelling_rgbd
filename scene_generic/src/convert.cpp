#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"

//typedef pcl::PointXYGRGBCam PointT;
typedef pcl::PointXYZRGBCamSL PointT;

int
  main (int argc, char** argv)
{ 
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<PointT> cloud;

  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  { 
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
  pcl::fromROSMsg (cloud_blob, cloud);

  std::string fn (argv[1]);
  fn = fn.substr(0,fn.find('.'));
  fn = fn+ "_binary.pcd";
  pcl::io::savePCDFile (fn, cloud, true);
  ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)cloud.points.size ());

  return (0);
}
