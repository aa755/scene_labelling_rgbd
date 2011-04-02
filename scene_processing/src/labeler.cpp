/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: segmentation_plane.cpp 35524 2011-01-26 08:20:18Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b segmentation_plane exemplifies how to run a Sample Consensus segmentation for planar models.

**/
#include <iostream>
#include <boost/thread.hpp>

#include <stdint.h>
//#include <pcl_visualization/cloud_viewer.h>
#include "pcl_visualization/pcl_visualizer.h"


#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;



typedef pcl::PointXYZRGBCamSL PointT;

//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;

void get_sorted_indices (pcl::PointCloud<PointT> &incloud , std::vector<int> &segmentindices , int size)
{
   std::map<int,int> countmap;
   for (int i = 1; i <=size; i++)
   {
     countmap[i] = 0;
   }
   for (size_t i = 0; i < incloud.points.size (); ++i)
   {
     countmap[incloud.points[i].segment ] = countmap[incloud.points[i].segment ] + 1;
   }
   std::multimap<int,int> inverted_countmap ;
   for ( std::map<int,int>::iterator it=countmap.begin() ; it != countmap.end(); it++ )
     inverted_countmap.insert (std::pair<int,int>(it->second ,it->first));
   for ( std::multimap<int,int>::reverse_iterator rit=inverted_countmap.rbegin(); rit != inverted_countmap.rend(); rit ++)
     segmentindices.push_back(rit->second);
     
}

void apply_segment_filter ( pcl::PointCloud<PointT> &incloud ,  pcl::PointCloud<PointT> &outcloud, pcl::PointCloud<PointT> &segment_cloud , int segment)
{
   ROS_INFO ("applying filter");
   outcloud.points.erase(outcloud.points.begin(),outcloud.points.end());
   segment_cloud.points.erase(segment_cloud.points.begin(),segment_cloud.points.end());
   outcloud.header.frame_id = incloud.header.frame_id;
   outcloud.points = incloud.points;
   segment_cloud.header.frame_id = incloud.header.frame_id;
   segment_cloud.points.resize ( incloud.points.size() );
   int j = 1;
   for (size_t i = 0; i < incloud.points.size (); ++i)
   {

     if(incloud.points[i].segment == segment) {
       segment_cloud.points[j].x = incloud.points[i].x;
       segment_cloud.points[j].y = incloud.points[i].y;
       segment_cloud.points[j].z = incloud.points[i].z;
       segment_cloud.points[j].rgb = incloud.points[i].rgb;
       segment_cloud.points[j].segment = incloud.points[i].segment;
       outcloud.points[i].rgb = 0.00005 ;
       j++;
     }
  }
   segment_cloud.points.resize ( outcloud.points.size() );
}

   pcl_visualization::PCLVisualizer viewer("3D Viewer");
//   int spin=1;
void spinThread()
{
    std::cerr<<"thread started";
    while(true)
        viewer.spinOnce(1000,true);
}
/* ---[ */
int
  main (int argc, char** argv)
{
    std::vector<std::string> labels;//(initLabels);
    std::ifstream labelFile;
    std::string line;
    labelFile.open("/opt/ros/unstable/stacks/scene_processing/labels.txt");

    if(labelFile.is_open())
    {
        int count=1;
        while(labelFile.good())
        {
            getline(labelFile,line);//each line is a label
            if(line.size()==0)
                break;
            cout<<"adding label "<<line<<" with value:"<<count<<endl;
            count++;
            labels.push_back(line);
        }
    }
    else
    {
        cout<<"could not open label file...exiting\n";
        exit(-1);
    }
  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2 cloud_blob_filtered;
  sensor_msgs::PointCloud2 cloud_blob_colored;

  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud2;
 //ColorHandlerPtr color_handler;
  pcl::PCDWriter writer;
  ColorHandlerPtr color_handler;

  pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr  cloud_colored (new pcl::PointCloud<PointT> ());

 
 // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file ");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1] ,pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blob, cloud);
   pcl::fromROSMsg (cloud_blob, cloud2);
   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
  
  // find the max segment number 
  int max_segment_num = 0;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
     if (max_segment_num < cloud.points[i].segment) {max_segment_num = cloud.points[i].segment;}    
  }
  std::map<int,int> label_mapping; 
  std::vector<int> segmentIndices; 
  get_sorted_indices ( *cloud_ptr , segmentIndices , max_segment_num);

  // get the 
    int viewport = 0;
    viewer.createViewPort(0.0,0.0,0.5,1.0,viewport);
    viewport=1;
    viewer.createViewPort(0.5,0.0,1.0,1.0,viewport);
  //for (int i = 1 ; i <= max_segment_num ; i++ ){
 
  for ( std::vector<int>::iterator it=segmentIndices.begin() ; it < segmentIndices.end(); it++ ) { 
    int i = *it;

   ROS_INFO ("CLuster number %d",i);
//    viewer.addCoordinateSystem(1.0f);

    apply_segment_filter( *cloud_ptr, *cloud_colored , *cloud_filtered, i);
    pcl::toROSMsg (*cloud_filtered,cloud_blob_filtered);
    pcl::toROSMsg (*cloud_colored,cloud_blob_colored);

    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_colored));
    viewer.addPointCloud(*cloud_colored,color_handler,"cloud",0);
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_filtered));
    viewer.addPointCloud(*cloud_filtered,color_handler ,"cluster",1);

  //  while (!viewer.wasStopped())

 //     viewer.spin();
      viewer.spinOnce(1000,true);

     // usleep(100000);
      char label[] = "8";
      int spintime=500;
      while(label[0]=='8')
      {
          cout << "Enter label(enter 8 if you want to see/iteract with the viewer for more time,9 to quit):" << endl;
//    if(it==segmentIndices.begin())
//        boost::thread trds(spinThread);
      //getline(cin, input_line);
        viewer.spinOnce(spintime,true);
          cin >> label;
          spintime=spintime*2; //show it for more time if user complains 

      }
      //if (strcmp (label, "q") == 0) {break;}
      if (label[0] =='9') {break;}
      bool done=false;
      while(!done)
      {
         cout <<"selected label:"<< label << endl;
         std::string labelStr(label);
         for(int li=0;li<labels.size();li++)
         {
             if(labelStr.compare(labels.at(li))==0)
             {
                label_mapping[i] = li+1;
                cout <<"label numerical alias:"<< li+1 << endl;

                done=true;
                break;
             }
         }
         if(!done)
         {
             cout <<"label not found... enter label again or 7 to add this as another label" << endl;
              cin >> label;
              if(label[0]=='7')
              {
                  cout<<"added new label:"<<labelStr<<endl;
                  labels.push_back(std::string(label));
                  //the next iteration will match and exit... no need to set done now
                  cout<<"new set of labels: \n";
                 for(int li=0;li<labels.size();li++)
                 {
                     cout<<labels.at(li)<<endl;
                 }

              }
             
         }
      }


    viewer.removePointCloud("cluster");
    viewer.removePointCloud("cloud");
 
 }




  std::vector<pcl::PointCloud<PointT> > clusters2;
  pcl::PointCloud<PointT> labeled_cloud;
  //getClustersFromPointCloud2(*cloud_filtered, clusters, clusters2,combined_cloud);
  labeled_cloud.header = cloud.header;
  labeled_cloud.points = cloud.points;
  for (size_t i = 0; i< labeled_cloud.points.size(); i++)
  {
    labeled_cloud.points[i].label = label_mapping[labeled_cloud.points[i].segment];

  }
  
  std::string fn = "labeled_"  + std::string(argv[1]);
    std::ofstream labelFileOut;
    labelFileOut.open("/opt/ros/unstable/stacks/scene_processing/labels.txt");
                 for(int li=0;li<labels.size();li++)
                 {
                     labelFileOut<<labels.at(li)<<endl;
                 }
    labelFileOut.close();

  writer.write ( fn,labeled_cloud, false);

  //cout<<"IMPORTANT!!! ... replace the first line after includes in labeler.cpp with:"


  return (0);
}
/* ]--- */
