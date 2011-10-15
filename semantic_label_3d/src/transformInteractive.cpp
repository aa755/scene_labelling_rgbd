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
#include <pcl/filters/passthrough.h>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <boost/thread.hpp>
#include <float.h>
#include <stdint.h>
//#include <pcl_visualization/cloud_viewer.h>
#include "pcl_visualization/pcl_visualizer.h"
//#include "../../combine_clouds/src/CombineUtils.h"


#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef pcl_visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2>::Ptr GeoHandlerPtr;
typedef ColorHandler::Ptr ColorHandlerPtr;



typedef pcl::PointXYZRGBCamSL PointT;

void
transformPointCloud (boost::numeric::ublas::matrix<double> &transform, pcl::PointCloud<PointT>::Ptr in,
                     pcl::PointCloud<PointT>::Ptr out)
{

  boost::numeric::ublas::matrix<double> matIn (4, 1);
  *out = *in;

  for (size_t i = 0; i < in->points.size (); ++i)
    {
      double * matrixPtr = matIn.data ().begin ();

      matrixPtr[0] = in->points[i].x;
      matrixPtr[1] = in->points[i].y;
      matrixPtr[2] = in->points[i].z;
      matrixPtr[3] = 1;
      boost::numeric::ublas::matrix<double> matOut = prod (transform, matIn);
      matrixPtr = matOut.data ().begin ();

      out->points[i].x = matrixPtr[0];
      out->points[i].y = matrixPtr[1];
      out->points[i].z = matrixPtr[2];
    }
}

/* ---[ */
int
main (int argc, char** argv)
{
  pcl_visualization::PCLVisualizer viewer ("3D Viewer");
  bool groundSelected = false;
  bool editLabel = false;
  int targetLabel;

  boost::numeric::ublas::matrix<double> outMat (4, 4);
  std::vector<std::string> labels; //(initLabels);
  std::ifstream labelFile;
  std::string line;
  sensor_msgs::PointCloud2 cloud_blob;

  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointT> cloudT;
  pcl::PointCloud<PointT>::Ptr labeled_transformed_cloud (new pcl::PointCloud<PointT > ());
  //ColorHandlerPtr color_handler;
  pcl::PCDWriter writer;
  ColorHandlerPtr color_handler;
  //  ColorHandlerPtr color_handlerX;
  //  ColorHandlerPtr color_handlerY;
  //  ColorHandlerPtr color_handlerY;
  GeoHandlerPtr geo_handler;



  // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
    {
      ROS_ERROR ("Couldn't read file ");
      return (-1);
    }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), argv[1], pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
  pcl::fromROSMsg (cloud_blob, cloud);
  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT > (cloud));
  pcl::PointCloud<PointT>::Ptr cloudT_ptr (new pcl::PointCloud<PointT > (cloudT));


  // find the max segment number 

  // get the 
  int viewportCloud = 0;
  int curLabel;

  // viewer.createViewPort(0.0,0.0,0.5,1.0,viewportCloud);
  //for (int i = 1 ; i <= max_segment_num ; i++ ){

  // color_handlerX.reset (new pcl_visualization::PointCloudColorHandl<sensor_msgs::PointCloud2 > (cloud_blob,"x"));
  color_handler.reset (new pcl_visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2 > (cloud_blob, "z"));
  viewer.addPointCloud (cloud, color_handler, "cloud");
  //viewer.

  //  while (!viewer.wasStopped())

  //     viewer.spin();
  viewer.addCoordinateSystem (1);
  char ans[20];
  outMat (3, 0) = 0;
  outMat (3, 1) = 0;
  outMat (3, 2) = 0;
  outMat (3, 3) = 1;
  while (true)
    {
      viewer.spinOnce (5000, true);
      viewer.resetCamera ();
      cin >> ans;
      char d = ans[0];
      double angle = atof (ans + 1);
      angle = angle * boost::math::constants::pi<double>() / 180.0;
      std::cerr << angle << endl;
      if (d == 'x')
        {

          outMat (0, 3) = 0;
          outMat (1, 3) = 0;
          outMat (2, 3) = 0;

          //set the 1st column=position of X axis
          outMat (0, 0) = 1;
          outMat (1, 0) = 0;
          outMat (2, 0) = 0;

          //set the 3rd column=position of Z axis
          outMat (0, 2) = 0;
          outMat (1, 2) = -sin (angle);
          outMat (2, 2) = cos (angle);




          //set the 2nd column=position of Y axis
          outMat (0, 1) = 0;
          outMat (1, 1) = cos (angle);
          outMat (2, 1) = sin (angle);
        }
      else if (d == 'y')
        {
          outMat (0, 3) = 0;
          outMat (1, 3) = 0;
          outMat (2, 3) = 0;

          //set the 1st column=position of X axis
          outMat (0, 0) = cos (angle);
          outMat (1, 0) = 0;
          outMat (2, 0) = sin (angle);

          //set the 2nd column=position of Y axis
          outMat (0, 1) = 0;
          outMat (1, 1) = 1;
          outMat (2, 1) = 0;

          //set the 3rd column=position of Z axis
          outMat (0, 2) = -sin (angle);
          outMat (1, 2) = 0;
          outMat (2, 2) = cos (angle);


        }
      else if (d == 's')
        {
          break;
        }
      else if (d == 'z')
        {
          pcl::toROSMsg (*cloud_ptr, cloud_blob);
          viewer.removePointCloud ();
          color_handler.reset (new pcl_visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2 > (cloud_blob, "z"));
          viewer.addPointCloud (*cloud_ptr, color_handler, "cloud");
          viewer.removeCoordinateSystem ();
          viewer.addCoordinateSystem (1);
          viewer.resetCamera ();
          continue;

        }
      else if (d == 'c')
        {
          pcl::toROSMsg (*cloud_ptr, cloud_blob);
          viewer.removePointCloud ();
          color_handler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
          viewer.addPointCloud (*cloud_ptr, color_handler, "cloud");
          viewer.removeCoordinateSystem ();
          viewer.addCoordinateSystem (1);
          viewer.resetCamera ();
          continue;
        }
      else if (d == 'r')
        {
          float minZ = angle / boost::math::constants::pi<double>()*180.0;
          pcl::PassThrough<PointT> pass;
          pass.setInputCloud (cloud_ptr);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (minZ, 10);
          pass.filter (*cloudT_ptr);
      pcl::toROSMsg (*cloudT_ptr, cloud_blob);

      viewer.removePointCloud ();
      //color_handler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
      color_handler.reset (new pcl_visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2 > (cloud_blob, "z"));
      viewer.addPointCloud (*cloudT_ptr, color_handler, "cloud");
      viewer.removeCoordinateSystem ();
      viewer.addCoordinateSystem (1);
      viewer.resetCamera ();
      *cloud_ptr = *cloudT_ptr;
      std::cerr<<"clipped pointcou dbelow "<<minZ<<endl;
      continue;
        }
      else
        continue;

      /* for(int i=0;i<4;i++)
       {
           for(int j=0;j<4;j++)
               std::cerr<<outMat(i,j)<<",";
           std::cerr<<endl;
       }       
       */
      transformPointCloud (outMat, cloud_ptr, cloudT_ptr);
      pcl::toROSMsg (*cloudT_ptr, cloud_blob);

      viewer.removePointCloud ();
      //color_handler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
      color_handler.reset (new pcl_visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2 > (cloud_blob, "z"));
      viewer.addPointCloud (*cloudT_ptr, color_handler, "cloud");
      viewer.removeCoordinateSystem ();
      viewer.addCoordinateSystem (1);
      viewer.resetCamera ();
      *cloud_ptr = *cloudT_ptr;



      // viewer.
      //                viewer.getPointCloudRenderingProperties (pcl_visualization::RenderingProperties)
      //viewer.
      /*              for(int i=0;i<3;i++)
                            std::cerr<<viewer.camera_.pos[i]<<" ";
                    std::cerr<<endl;
                
                    for(int i=0;i<3;i++)
                            std::cerr<<viewer.camera_.focal[i]<<" ";
                    std::cerr<<endl;

                    for(int i=0;i<3;i++)
                            std::cerr<<viewer.camera_.view[i]<<" ";
                    std::cerr<<"\n--------"<<endl;
       */
    }

  // usleep(100000);



  //make z(ground)=0
  float minZ = FLT_MAX;
  float z;
  for (int i = 1; i < cloud_ptr->size (); i++)
    {
      z = cloud_ptr->points[i].z;
      if (z < minZ)
        minZ = z;
    }

  for (int i = 0; i < cloud_ptr->size (); i++)
    {
      cloud_ptr->points[i].z -= minZ;
    }
  std::cerr << "minZ was " << minZ << endl;

  std::string fn (argv[1]);
  writer.write ("transformed_" + fn, *cloud_ptr, true);
}
/* ]--- */
