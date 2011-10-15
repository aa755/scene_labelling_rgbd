/* 
 * File:   genImagePairs.cpp
 * Author: aa755
 *
 * Created on May 10, 2011, 10:12 PM
 */

#include <iostream>
#include <boost/thread.hpp>

#include <stdint.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>



#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl_ros/io/bag_io.h>

#include "pcl_visualization/pcl_visualizer.h"
#include <dynamic_reconfigure/server.h>
#include <scene_processing/pcmergerConfig.h>
#include "transformation.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "includes/color.cpp"
#include "HOG.cpp"

typedef pcl::PointXYZRGBCamSL PointT;
//#include "includes/CombineUtils.h"
//#include "includes/CombineUtils.h"

void saveFloatImage ( const char* filename, const IplImage * image )
{
  IplImage * saveImage = cvCreateImage ( cvGetSize ( image ),
                                             IPL_DEPTH_32F, 3 );
  cvConvertScale ( image, saveImage, 255, 0 );
  cvSaveImage( filename, saveImage);
  cvReleaseImage ( &saveImage );
}

/*
 * 
 */
int
main (int argc, char** argv)
{
  std::string fn;

  int frameNumber=0;

  //sensor_msgs::PointCloud2 cloud_blob_new;
  //sensor_msgs::PointCloud2 cloud_blob_prev;
  sensor_msgs::PointCloud2 cloud_blob_merged;
  sensor_msgs::PointCloud2ConstPtr cloud_blob_new, cloud_blob_prev;
  //sensor_msgs::Image image;
 
 

  pcl::PointCloud<PointT> cloud_new;
  pcl::PCDWriter writer;



  pcl::PointCloud<PointT>::Ptr cloud_prev_ptr (new pcl::PointCloud<PointT > ());
  //pcl::PointCloud<PointT>::Ptr cloud_new_ptr (new pcl::PointCloud<PointT > ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new_ptr (new pcl::PointCloud<pcl::PointXYZRGB > ());
  pcl::PointCloud<PointT>::Ptr cloud_mod_ptr (new pcl::PointCloud<PointT > ());
  pcl::PointCloud<PointT>::Ptr cloud_merged_ptr (new pcl::PointCloud<PointT > ());
  pcl::PointCloud<PointT>::Ptr cloud_merged_backup_ptr (new pcl::PointCloud<PointT > ());
  bool doUpdate = false;
  bool Merged = false;
  bool ITpresent = false;
  pcl_ros::BAGReader reader;
  int skipNum = 20;

  scene_processing::pcmergerConfig InitialTransformConfig;
  char *topic = "/camera/rgb/points";
  if (argc > 3)
    frameNumber = atoi(argv[3]);

  if (!reader.open (argv[1], "/rgbdslam/my_clouds"))
    {
      cout << "Couldn't read bag file on topic" << (topic);
      return (-1);
    }
  cout<<frameNumber<<endl;
  for(int i=0;i<5*frameNumber;i++)
    reader.getNextCloud ();
  
  cloud_blob_new = reader.getNextCloud ();
  pcl::fromROSMsg (*cloud_blob_new, *cloud_new_ptr);

  if (pcl::io::loadPCDFile (argv[2], cloud_blob_merged) == -1)
    {
      ROS_ERROR ("Couldn't read file test_pcd.pcd");
      return (-1);
    }
  //    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

  pcl::fromROSMsg (cloud_blob_merged, *cloud_merged_ptr);

  std::map<int,int> labels;
 /* for (int i = 0; i < cloud_merged_ptr->size (); i++)
    {
      if (cloud_merged_ptr->points[i].cameraIndex == frameNumber)
        {
          PointT src;
          pcl::PointXYZRGB tmp;
          src = cloud_merged_ptr->points[i];
          int count = 0;
          for (int j = 0; j < cloud_new_ptr->size (); j++)
            {
              tmp = cloud_new_ptr->points[j];
              if (src.x == tmp.x && src.y == tmp.y && src.z == tmp.z && src.rgb == tmp.rgb)
                {
                  labels[j] = 1;//src.label;
                  count++;
                }
            }
          //cout<<count<<endl;
          assert (count == 1);
        }   
    }
*/
  CvSize size;
  size.height=480;
  size.width=640;
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
        ColorRGB *labelColors[9];
        labelColors[0]= new ColorRGB(1,0,0);
        labelColors[1]= new ColorRGB(0,1,0);
        labelColors[2]= new ColorRGB(0,0,1);
        labelColors[3]= new ColorRGB(1,1,0);
        labelColors[4]= new ColorRGB(0,1,1);
        labelColors[5]= new ColorRGB(1,0,1);
        labelColors[6]= new ColorRGB(0.5,0,0);
        labelColors[7]= new ColorRGB(0,0.5,0);
        labelColors[8]= new ColorRGB(0,0,0.5);  
          pcl::PointXYZRGB tmp;
  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++)
      {
        int index=x+y*size.width;
        tmp= cloud_new_ptr->points[index];
        ColorRGB tmpColor(tmp.rgb);
        if(labels[index]>=1 && labels[index]<=9)
          {
           // tmpColor=*labelColors[labels[index]-1];
          }
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }

saveFloatImage ( "snapshot.png", image );
HOG hog;
//Point2D
//hog.computeHog (image);
//hog.saveFeatAsImages ();
  return 0;
}

