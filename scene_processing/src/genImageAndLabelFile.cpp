/* 
 * File:   colorImage.cpp
 * Author: aa755
 *
 * Created on June 23, 2011, 2:21 PM
 */

/* 
 * File:   applyLabels.cpp
 * Author: aa755
 *
 * Created on April 23, 2011, 11:26 PM
 */

#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
//#include <X11/X.h>
//#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"

#include "sensor_msgs/point_cloud_conversion.h"
#include "color.cpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "HOG.cpp"

typedef pcl::PointXYZRGBCamSL PointT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;


using namespace pcl;


int main(int argc, char** argv) {

  if(argc!=2)
    {
        cerr<<"usage: "<<argv[1]<<" pointCloudFile"<<endl;
        exit(-1);
    }
  
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;
    // read the pcd file

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read the input file ");
        return (-1);
    }
    //ROS_INFO("Loaded %d data points from "+string(argv[1])+" with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

   pcl::fromROSMsg (cloud_blob, cloud);
          char filename[100];
          sprintf(filename,"%s.txt",argv[1]);
   std::ofstream labelCsv;
   labelCsv.open(filename);
  

  CvSize size;
  size.height=480;
  size.width=640;
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
          pcl::PointXYZRGBCamSL tmp;
          
  for(int y=0;y<size.height;y++)
  {
      for(int x=0;x<size.width;x++)
      {
        int index=x+y*size.width;
        tmp= cloud.points[index];
            ColorRGB tmpColor(tmp.rgb);
            
        labelCsv<<tmp.label;      
        
        if(x==size.width-1)
            labelCsv<<"\n";
        else
            labelCsv<<"\t";
        
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }
  }       

          sprintf(filename,"%s.png",argv[1]);
          labelCsv.close();
HOG::saveFloatImage ( filename, image );
cvReleaseImage (&image);


}


