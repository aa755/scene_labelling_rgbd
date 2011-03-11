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

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;


namespace my_ns
{
    struct MyPoint
    { 
       float x;
       float y;
       float z;
       float rgb;
       uint32_t segment;
       uint32_t label;

    };
    


} // namespace my_ns

POINT_CLOUD_REGISTER_POINT_STRUCT(
      my_ns::MyPoint,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, rgb, rgb)
      (uint32_t, segment, segment)
      (uint32_t, label, label)
);




typedef my_ns::MyPoint PointT;
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

void apply_segment_filter ( pcl::PointCloud<my_ns::MyPoint> &incloud ,  pcl::PointCloud<my_ns::MyPoint> &outcloud, pcl::PointCloud<my_ns::MyPoint> &segment_cloud , int segment)
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
/* ---[ */
int
  main (int argc, char** argv)
{


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
  //for (int i = 1 ; i <= max_segment_num ; i++ ){ 
  for ( std::vector<int>::iterator it=segmentIndices.begin() ; it < segmentIndices.end(); it++ ) { 
    int i = *it;

   ROS_INFO ("CLuster number %d",i);
   pcl_visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.addCoordinateSystem(1.0f);

    apply_segment_filter( *cloud_ptr, *cloud_colored , *cloud_filtered, i);
    pcl::toROSMsg (*cloud_filtered,cloud_blob_filtered);
    pcl::toROSMsg (*cloud_colored,cloud_blob_colored);

    int viewport = 0;
    viewer.createViewPort(0.0,0.0,0.5,1.0,viewport);
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_colored));
    viewer.addPointCloud(*cloud_colored,color_handler,"cloud",viewport);
    viewer.createViewPort(0.5,0.0,1.0,1.0,viewport);
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_filtered));
    viewer.addPointCloud(*cloud_filtered,color_handler ,"cluster",viewport);

  //  while (!viewer.wasStopped())

      viewer.spin();
     // usleep(100000);
      char label[] = "0";
      cout << "Enter label:" << endl;
      //getline(cin, input_line);
      cin >> label;
      cout << label << endl;
      if (strcmp (label, "q") == 0) {break;}
      label_mapping[i] = atoi(label);
//     }

    viewer.removePointCloud("cluster");
    viewer.removePointCloud("cloud");
 
 }




  std::vector<pcl::PointCloud<my_ns::MyPoint> > clusters2;
  pcl::PointCloud<my_ns::MyPoint> labeled_cloud;
  //getClustersFromPointCloud2(*cloud_filtered, clusters, clusters2,combined_cloud);
  labeled_cloud.header = cloud.header;
  labeled_cloud.points = cloud.points;
  for (size_t i = 0; i< labeled_cloud.points.size(); i++)
  {
    labeled_cloud.points[i].label = label_mapping[labeled_cloud.points[i].segment];

  }
  
  //std::string fn = "labeled_" + argv[1];
  writer.write ( argv[2],labeled_cloud, false);



  return (0);
}
/* ]--- */
