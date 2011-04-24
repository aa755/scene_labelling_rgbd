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
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"

#include "sensor_msgs/point_cloud_conversion.h"
#include "color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"


typedef pcl::PointXYZRGBCamSL PointT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;


using namespace pcl;


int main(int argc, char** argv) {
    pcl::PCDWriter writer;

  if(argc!=3)
    cout<<"usage: "<<argv[1]<<" pointCloudFile segment2labelMappingFile"<<endl;
  std::map<int,int> label_mapping; 

    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;
    // read the pcd file
    std::ifstream labelFile;
    labelFile.open(argv[2]);
    int segNo, label;
        if(labelFile.is_open())
    {
        int count=1;
        while(labelFile.good())
        {
            label=0;
            labelFile>>segNo>>label;
            if(label==0)
                break;
            cout<<"adding seg "<<segNo<<" with label:"<<label<<endl;
            label_mapping[segNo]=label;
        }
    }
    else
    {
        cout<<"could not open label file...exiting\n";
        exit(-1);
    }


    

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

   pcl::fromROSMsg (cloud_blob, cloud);



  pcl::PointCloud<PointT> labeled_cloud;
  labeled_cloud.header = cloud.header;
  labeled_cloud.points = cloud.points;
  for (size_t i = 0; i< labeled_cloud.points.size(); i++)
  {
    labeled_cloud.points[i].label = label_mapping[labeled_cloud.points[i].segment];

  }
std::string      fn = "pred_labeled_"  + std::string(argv[1]);

  writer.write ( fn,labeled_cloud, true);
}


