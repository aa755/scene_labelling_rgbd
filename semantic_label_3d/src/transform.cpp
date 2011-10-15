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
typedef ColorHandler::Ptr ColorHandlerPtr;



typedef pcl::PointXYZRGBCamSL PointT;

float
sqrG (float y)
{
  return y*y;
}
//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;

class VectorG
{
public:
  double v[3];

  VectorG () { }

  VectorG (double unitX, double unitY, double unitZ)
  {
    v[0] = unitX;
    v[1] = unitY;
    v[2] = unitZ;


  }

  bool
  isUnitVector ()
  {
    return (isZero (getNormSqr () - 1));

  }

  VectorG (PointT p)
  {
    v[0] = p.x;
    v[1] = p.y;
    v[2] = p.z;


  }

  /**
   * compute the distance from the line joining p1 and p2
   */
  double
  computeDistanceSqrFromLine (VectorG p1, VectorG p2)
  {
    VectorG t1 = p1.subtract (*this);

    VectorG t2 = p2.subtract (p1);

    return (t1.getNormSqr () * t2.getNormSqr () - sqrG (t1.dotProduct (t2))) / t2.getNormSqr ();
  }

  /**
   *true iff it lies in the cylinder of infinite radius defined by p1 and p2
   * the line segment (p1,p2) is the axis of this cylinder
   */
  bool
  isInsideLineSegment (VectorG p1, VectorG p2)
  {
    VectorG lineSeg = p1.subtract (p2);
    double lengthSqr = lineSeg.getNormSqr ();
    VectorG p1Seg = subtract (p1);
    VectorG p2Seg = subtract (p2);

    if (p1Seg.getNormSqr () <= lengthSqr && p2Seg.getNormSqr () <= lengthSqr)
      return true;
    else
      return false;


  }

  void
  normalize ()
  {
    double norm = getNorm ();
    for (int i = 0; i < 3; i++)
      v[i] = v[i] / norm;
  }

  PointT
  getAsPoint ()
  {
    PointT p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
  }

  double
  getNorm ()
  {
    return sqrt (getNormSqr ());
  }

  double
  getNormSqr ()
  {
    return (sqrG (v[0]) + sqrG (v[1]) + sqrG (v[2]));
  }

  double
  dotProduct (VectorG v2g)
  {
    double sum = 0;
    for (int i = 0; i < 3; i++)
      sum = sum + v[i] * v2g.v[i];
    return sum;
  }

  static bool
  isZero (double d)
  {
    std::cerr << "value is:" << d << endl;
    return (fabs (d) < 0.001);
  }
  //(a2b3 − a3b2, a3b1 − a1b3, a1b2 − a2b1)

  VectorG
  crossProduct (VectorG v2g)
  {
    VectorG ret (v[2 - 1] * v2g.v[3 - 1] - v[3 - 1] * v2g.v[2 - 1], v[3 - 1] * v2g.v[1 - 1] - v[1 - 1] * v2g.v[3 - 1], v[0] * v2g.v[2 - 1] - v[2 - 1] * v2g.v[1 - 1]);
    std::cerr << "norm is" << ret.getNorm () << endl;
    assert (isZero (ret.getNorm () - 1));
    assert (isZero (ret.dotProduct (*this)));
    assert (isZero (ret.dotProduct (v2g)));
    return ret;
  }

  VectorG
  multiply (double scalar)
  {
    VectorG out;
    for (int i = 0; i < 3; i++)
      out.v[i] = scalar * v[i];
    return out;
  }

  VectorG
  subtract (VectorG v2g)
  {
    VectorG out;
    for (int i = 0; i < 3; i++)
      out.v[i] = v[i] - v2g.v[i];
    return out;

  }

  VectorG
  add (VectorG v2g)
  {
    VectorG out;
    for (int i = 0; i < 3; i++)
      out.v[i] = v[i] + v2g.v[i];
    return out;

  }

  float
  eucliedianDistance (VectorG v2g)
  {
    float sum = 0;
    for (int i = 0; i < 3; i++)
      sum = sum + sqrG (v[i] - v2g.v[i]);
    return sqrt (sum);

  }

};

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

void
get_sorted_indices (pcl::PointCloud<PointT> &incloud, std::vector<int> &segmentindices, int size)
{
  std::map<int, int> countmap;
  for (int i = 1; i <= size; i++)
    {
      countmap[i] = 0;
    }
  for (size_t i = 0; i < incloud.points.size (); ++i)
    {
      countmap[incloud.points[i].segment ] = countmap[incloud.points[i].segment ] + 1;
    }
  std::multimap<int, int> inverted_countmap;
  for (std::map<int, int>::iterator it = countmap.begin (); it != countmap.end (); it++)
    inverted_countmap.insert (std::pair<int, int>(it->second, it->first));
  for (std::multimap<int, int>::reverse_iterator rit = inverted_countmap.rbegin (); rit != inverted_countmap.rend (); rit++)
    segmentindices.push_back (rit->second);

}

void
apply_segment_filter (pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, pcl::PointCloud<PointT> &segment_cloud, int segment)
{
  ROS_INFO ("applying filter");
  outcloud.points.erase (outcloud.points.begin (), outcloud.points.end ());
  segment_cloud.points.erase (segment_cloud.points.begin (), segment_cloud.points.end ());
  outcloud.header.frame_id = incloud.header.frame_id;
  outcloud.points = incloud.points;
  segment_cloud.header.frame_id = incloud.header.frame_id;
  segment_cloud.points.resize (incloud.points.size ());
  int j = 0;
  for (size_t i = 1; i < incloud.points.size (); ++i)
    {

      if (incloud.points[i].segment == segment)
        {
          segment_cloud.points[j].x = incloud.points[i].x;
          segment_cloud.points[j].y = incloud.points[i].y;
          segment_cloud.points[j].z = incloud.points[i].z;
          segment_cloud.points[j].rgb = incloud.points[i].rgb;
          segment_cloud.points[j].segment = incloud.points[i].segment;
          segment_cloud.points[j].label = incloud.points[i].label;
          //     std::cerr<<segment_cloud.points[j].label<<",";
          outcloud.points[i].rgb = 0.00005;
          j++;
        }
    }
  segment_cloud.points.resize (outcloud.points.size ());
}

//pcl_visualization::PCLVisualizer viewer("3D Viewer");
//   int spin=1;

/* ---[ */
int
main (int argc, char** argv)
{
  bool groundSelected = false;
  bool editLabel = false;
  int targetLabel;
  if (argc > 2)
    {
      editLabel = true;
      targetLabel = atoi (argv[2]);
    }

  boost::numeric::ublas::matrix<double> outMat (4, 4);
  std::vector<std::string> labels; //(initLabels);
  std::ifstream labelFile;
  std::string line;
  labelFile.open ("/opt/ros/unstable/stacks/scene_processing/labels.txt");

  std::cerr << "you can only quit by pressing 9 when the prompt mentions... quitting in other ways will discard any newly added labels\n";
  if (labelFile.is_open ())
    {
      int count = 1;
      while (labelFile.good ())
        {
          getline (labelFile, line); //each line is a label
          if (line.size () == 0)
            break;
          cout << "adding label " << line << " with value:" << count << endl;
          count++;
          labels.push_back (line);
        }
    }
  else
    {
      cout << "could not open label file...exiting\n";
      exit (-1);
    }
  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2 cloud_blob_filtered;
  sensor_msgs::PointCloud2 cloud_blob_colored;

  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointT>::Ptr labeled_transformed_cloud (new pcl::PointCloud<PointT > ());
  //ColorHandlerPtr color_handler;
  pcl::PCDWriter writer;
  ColorHandlerPtr color_handler;

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT > ());
  pcl::PointCloud<PointT>::Ptr cloud_colored (new pcl::PointCloud<PointT > ());


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


  // find the max segment number 
  int max_segment_num = 0;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (max_segment_num < cloud.points[i].segment)
        {
          max_segment_num = cloud.points[i].segment;
        }
    }
  std::map<int, int> label_mapping;
  std::vector<int> segmentIndices;
  get_sorted_indices (*cloud_ptr, segmentIndices, max_segment_num);

  // get the 
  int viewportCloud = 0;
  int viewportCluster = 0;
  int curLabel;

  //   viewer.createViewPort(0.0,0.0,0.5,1.0,viewportCloud);
  //for (int i = 1 ; i <= max_segment_num ; i++ ){
  int segNum=0;
  for (std::vector<int>::iterator it = segmentIndices.begin (); it < segmentIndices.end (); it++)
    {
      int i = *it;
      segNum++;

      ROS_INFO ("CLuster number %d", i);
      //    viewer.addCoordinateSystem(1.0f);

      apply_segment_filter (*cloud_ptr, *cloud_colored, *cloud_filtered, i);
      /*      for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        {
           std::cerr <<  i << ": " << cloud.points[i].segment << ", " << cloud.points[i].label << std::endl;
        }
       */
      curLabel = cloud_filtered->points[1].label;
      pcl::toROSMsg (*cloud_filtered, cloud_blob_filtered);
      pcl::toROSMsg (*cloud_colored, cloud_blob_colored);

      //   color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_colored));
      //    viewer.addPointCloud(*cloud_colored,color_handler,"cloud",viewportCloud);
      //viewer.

      //  while (!viewer.wasStopped())

      //     viewer.spin();
      //      viewer.spinOnce(1000,true);

      // usleep(100000);

      if(segNum>15)
        {
          std::cerr<<"none of top 14 segs were ground ..exiting\n";
          break;
        }
      if (curLabel == 2)
        {
          pcl::ModelCoefficients coefficients;
          pcl::PointIndices inliers;
          // Create the segmentation object
          pcl::SACSegmentation<PointT> seg;
          // Optional
          seg.setOptimizeCoefficients (true);
          // Mandatory
          seg.setModelType (pcl::SACMODEL_PLANE);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setDistanceThreshold (100);
          seg.setMaxIterations (1000000);

          //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ > (cloud));
          seg.setInputCloud (cloud_filtered);
          seg.segment (inliers, coefficients);

          if (inliers.indices.size () == 0)
            {
              ROS_ERROR ("Could not estimate a planar model for the given dataset.");
              return (-1);
            }
          groundSelected = true;
          std::cerr << "Model coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " "
                  << coefficients.values[2] << " " << coefficients.values[3] << std::endl;

          int numPointsInFloor = cloud_filtered->points.size ();
          VectorG sum (0, 0, 0);
          for (int fili = 0; fili < numPointsInFloor; ++fili)
            {
              sum = sum.add (VectorG (cloud_filtered->points[fili].x, cloud_filtered->points[fili].y, cloud_filtered->points[fili].z));
            }
          VectorG origin = sum.multiply (1.0 / numPointsInFloor);
          std::cerr << "origin:" << origin.v[0] << "," << origin.v[1] << "," << origin.v[2] << endl;
//          origin.v[2] = -(coefficients.values[0]/*a*/ * origin.v[0] + coefficients.values[1]/*b*/ * origin.v[1] + coefficients.values[3]/*d*/) / coefficients.values[2]/*c*/;

          double cosTheta = coefficients.values[3];
          double sinTheta = sqrt (1 - cosTheta * cosTheta);
          double root2 = sqrt (2);
          outMat (3, 3) = 1;
          //set the  displacements:
          outMat (0, 3) = -origin.v[0];
          outMat (1, 3) = -origin.v[1];
          outMat (2, 3) = -origin.v[2];
          VectorG vz (coefficients.values[0], coefficients.values[1], coefficients.values[2]);
          assert (vz.isUnitVector ());
          assert (VectorG::isZero (vz.dotProduct (origin) + coefficients.values[3]));
          //set the 3rd column=position of Z axis
          outMat (0, 2) = coefficients.values[0];
          outMat (1, 2) = coefficients.values[1];
          outMat (2, 2) = coefficients.values[2];

          //The plane coefficients are: a, b, c, d (ax+by+cz+d=0)
          double tx = 5;
          double ty = 0;
          double tz = -(coefficients.values[0]/*a*/ * tx + coefficients.values[3]/*d*/) / coefficients.values[2]/*c*/;
          VectorG v1 (tx, ty, tz);
          VectorG v2 (0, 0, - coefficients.values[3]/*d*/ / coefficients.values[2]/*c*/);
          VectorG vx = v1.subtract (v2);
          vx.normalize ();
          assert (VectorG::isZero (vx.dotProduct (vz)));

          //set the 1st column=position of X axis
          outMat (0, 0) = vx.v[0];
          outMat (1, 0) = vx.v[1];
          outMat (2, 0) = vx.v[2];

          VectorG vy = vz.crossProduct (vx);

          //set the 2nd column=position of Y axis
          outMat (0, 1) = vy.v[0];
          outMat (1, 1) = vy.v[1];
          outMat (2, 1) = vy.v[2];

          outMat (3, 0) = 0;
          outMat (3, 1) = 0;
          outMat (3, 2) = 0;

            break; 

          //The plane coefficients are: a, b, c, d (ax+by+cz+d=0)


        }

    }




  std::string fn (argv[1]);
  if(groundSelected)
    {
  transformPointCloud (outMat, cloud_ptr, labeled_transformed_cloud);
  //  writer.write ( fn,labeled_cloud, true);
  
  writer.write ("transformed_" + fn, *labeled_transformed_cloud, true);
    }
  return (0);
}
/* ]--- */
