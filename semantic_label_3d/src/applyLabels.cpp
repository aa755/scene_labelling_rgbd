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
#include "includes/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"

#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"


typedef pcl::PointXYZRGBCamSL PointT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;


using namespace pcl;


int main(int argc, char** argv) {
    pcl::PCDWriter writer;

  if(argc!=3&&argc!=4)
    {
    cerr<<"usage: "<<argv[1]<<" pointCloudFile segment2labelMappingFile [evaluate]"<<endl;
    exit(-1);
    }

bool evaluate=false;

if(argc==4)
	evaluate=true;

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
            label=-1;
            labelFile>>segNo>>label;
            if(label==-1)
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
if(!evaluate)
{
  labeled_cloud.header = cloud.header;
  labeled_cloud.points = cloud.points;
}

  int NUM_CLASSES=5;
  int tp[NUM_CLASSES];
  int tc[NUM_CLASSES];
  int pc[NUM_CLASSES];
  
  for(int i=0;i<5;i++)
  {
      tp[i]=0;
      tc[i]=0;
      pc[i]=0;
  }
  
  int countTrue=0;
  int countFalse=0;
  for (size_t i = 0; i< cloud.points.size(); i++)
  {
     //if(label_mapping[labeled_cloud.points[i].segment]!=0)
	if(evaluate&&cloud.points[i].label>0)
	{
            tc[cloud.points[i].label-1]++;
            if(label_mapping[cloud.points[i].segment]>0)
            {
                pc[label_mapping[cloud.points[i].segment]-1]++;
            }
		if(cloud.points[i].label == label_mapping[cloud.points[i].segment])
                {
			countTrue++;
                        tp[cloud.points[i].label-1]++;
                }
		else
			countFalse++;
	}
	else if(!evaluate)
        	labeled_cloud.points[i].label = label_mapping[labeled_cloud.points[i].segment];

  }

if(evaluate)
{

    std::ofstream nfeatfile, efeatfile;
    nfeatfile.open("res.txt",ios::app);
    nfeatfile<<countTrue<<" "<<countFalse<<" "<<cloud.size();//<<endl;
    for(int i=0;i<NUM_CLASSES;i++)
        nfeatfile<<" "<<tp[i]<<" "<<tc[i]<<" "<<pc[i];
    nfeatfile<<endl;
}
else
{
std::string      fn = "pred_labeled_"  + std::string(argv[1]);
writer.write ( fn,labeled_cloud, true);
}
}


