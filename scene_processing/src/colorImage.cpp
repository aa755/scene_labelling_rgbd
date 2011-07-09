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

  bool colorLabels=true;
  if(argc<3)
    {
    cerr<<"usage: "<<argv[1]<<" pointCloudFile label2color [NoLabels]"<<endl;
    exit(-1);
    }
  if(argc==4)
      colorLabels=false;
  
  std::map<int,int> color_mapping; 

    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;
    // read the pcd file
    std::ifstream labelFile;
    labelFile.open(argv[2]);
    int labelNum, colorNum;
        if(labelFile.is_open())
    {
        int count=1;
        while(labelFile.good())
        {
            colorNum=0;
            labelFile>>colorNum>>labelNum;
            if(colorNum==0)
                break;
           // cout<<"adding seg "<<labelNum<<" with label:"<<colorNum<<endl;
            color_mapping[labelNum]=colorNum;
        }
    }
    else
    {
        cout<<"could not open color mapping file...exiting\n";
        exit(-1);
    }
    int NUM_CLASSES_TO_SHOW=10;
        ColorRGB *labelColors[NUM_CLASSES_TO_SHOW];
        labelColors[0]= new ColorRGB(1,0,0);
        labelColors[1]= new ColorRGB(0,1,0);
        labelColors[2]= new ColorRGB(0,0,1);
        labelColors[3]= new ColorRGB(1,1,0);
        labelColors[4]= new ColorRGB(0,1,1);
        labelColors[5]= new ColorRGB(1,0,1);
        labelColors[6]= new ColorRGB(0.5,0,0);
        labelColors[7]= new ColorRGB(0,0.5,0);
        labelColors[8]= new ColorRGB(0,0,0.5);
        labelColors[9]= new ColorRGB(0.5,0,0.5);

    

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

   pcl::fromROSMsg (cloud_blob, cloud);



  CvSize size;
  size.height=480;
  size.width=640;
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
          pcl::PointXYZRGBCamSL tmp;
  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++)
      {
        int index=x+y*size.width;
        tmp= cloud.points[index];
            ColorRGB tmpColor(tmp.rgb);
            
        if(colorLabels&&tmp.label>0&&color_mapping[tmp.label]>0)
        {
           tmpColor=*labelColors[color_mapping[tmp.label]-1]; 
        }
            
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }
          

          char filename[100];
          sprintf(filename,"%s.png",argv[1]);
HOG::saveFloatImage ( filename, image );
cvReleaseImage (&image);

}


