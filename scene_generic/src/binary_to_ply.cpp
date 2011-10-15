#include <fstream>

#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "pcl/filters/passthrough.h"
#include "includes/color.cpp"

//typedef pcl::PointXYGRGBCam PointT;
typedef pcl::PointXYZRGBCamSL  PointT;


void writePLY(std::string filename, std::vector<pcl::PointPLY> points){
    ofstream myfile;
    myfile.open(filename.c_str());
    
    char header[300] ;
    sprintf(header, "ply\nformat ascii 1.0\ncomment author: Hema\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n", points.size());
    myfile << header ;
    for(size_t i = 0; i < points.size(); i++)
    {
        myfile << points.at(i).x << " " << points.at(i).y << " " << points.at(i).z << " " << points.at(i).r << " " << points.at(i).g << " " << points.at(i).b << endl;
    }
    myfile.close();
            
}

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
  pcl::PointCloud<PointT>::Ptr  cloud_ptr (new pcl::PointCloud<PointT> (cloud));
  
  
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud_ptr);
  pass.filter (cloud);
  
  ROS_INFO ("Size after removing NANs : %d", (int)cloud.points.size());
  
  ColorRGB color;
  vector<pcl::PointPLY> points (cloud.points.size());
  for(size_t i = 0; i < cloud.points.size(); i++)
  {
      color.assignColor(cloud.points.at(i).rgb);
      points.at(i).x = cloud.points.at(i).x;
      points.at(i).y = cloud.points.at(i).y;
      points.at(i).z = cloud.points.at(i).z;
      points.at(i).r = color.getR();
      points.at(i).g = color.getG();
      points.at(i).b = color.getB();
      
  }
  
  std::string fn (argv[1]);
  fn = fn.substr(0,fn.find('.'));
  fn = fn+ ".ply";
 // pcl::io::savePCDFileASCII (fn, cloud);
  writePLY(fn,points);
  ROS_INFO ("Saved %d data points to ply file.", (int)cloud.points.size ());

  return (0);
}
