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
 * $Id: bag_read.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b bag_read exemplifies how to read sensor_msgs/PointCloud2 messages from a BAG file, and convert them to templated PointClouds.

**/


/*
namespace my_ns
{
    struct MyPoint
    { 
       float x;
       float y;
       float z;
       uint32_t rgb;
       uint32_t index;
    };
} // namespace my_ns

// Must be used in the global namespace with the fully-qualified name
// of the point type.
POINT_CLOUD_REGISTER_POINT_STRUCT(
      my_ns::MyPoint,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, rgb, rgb)
      (uint32_t, index, index));

*/

#include <stdint.h>
#include "pcl/ros/register_point_struct.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include "pcl/io/pcd_io.h"
#include <string>
#include <pcl_ros/io/bag_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pcl/ModelCoefficients.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;
typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;

/* ---[ */
int
  main (int argc, char** argv)
{
  sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
  //pcl::PointCloud<pcl::PointXYZ> cloud, combined_cloud;
  pcl::PointCloud<PointT> cloud;// combined_cloud;
  pcl::PointCloud<PointT>::Ptr  combined_cloud_ptr (new pcl::PointCloud<PointT> () );
  float tolerance = 0.005; 
  pcl::PassThrough<PointT> pass_;

  pcl_ros::BAGReader reader;
  char *topic="/rgbdslam/transformed_cloud";
  if(argc>2)
	topic=argv[2];
std::cerr<<topic<<std::endl;
  if (!reader.open (argv[1], topic))
  {
    ROS_ERROR ("Couldn't read file ");
    return (-1);
  }

  int cnt = 0;
  do
  {
    cloud_blob_prev = cloud_blob;
    cloud_blob = reader.getNextCloud ();
    if (cloud_blob_prev != cloud_blob)
    {
      pcl::fromROSMsg (*cloud_blob, cloud);
      
      ROS_INFO ("PointCloud with %d data points and frame %s (%f) received.", (int)cloud.points.size (), cloud.header.frame_id.c_str (), cloud.header.stamp.toSec ()); 
      cnt++;
    }

    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
    pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT> ());
    pass_.setInputCloud (cloud_ptr);
    pass_.filter (*cloud_filtered);

    if(cnt == 1)  *combined_cloud_ptr = *cloud_filtered;

    else if (cnt%5 == 1) // combine every 5th frame
    {
      ROS_INFO ("Starting the merge..");
      pcl::PointCloud<PointT> new_cloud;
      new_cloud.points.resize(cloud_filtered->points.size());
      new_cloud.header = cloud_filtered->header;
      KdTreePtr  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
      initTree (0, clusters_tree_);
      clusters_tree_->setInputCloud (combined_cloud_ptr );
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;
      int nr_p = 0;
      ROS_INFO ("Input cloud size = %d",  cloud_filtered->points.size() );
      for (size_t  i =0; i < cloud_filtered->points.size(); ++i ) 
      {
        
        if (!clusters_tree_->radiusSearch (cloud_filtered->points[i], tolerance, nn_indices, nn_distances))
        {
          new_cloud.points[nr_p++] = cloud_filtered->points[i];
          
        }else {
        }
        
      }
      new_cloud.points.resize(nr_p);
      ROS_INFO ("New cloud size = %d",  new_cloud.points.size() );
      *combined_cloud_ptr += new_cloud;
      ROS_INFO ("merged cloud size = %d",  combined_cloud_ptr->points.size() );
      ROS_INFO ("merge complete" );
    }
    std::stringstream ss;
    ss << cnt;
    std::string fn = "test"+ ss.str() + ".pcd";

    pcl::io::savePCDFile (fn, *cloud_filtered, true);
    ROS_INFO ("Saved %d data points to file.", (int)cloud_filtered->points.size ());

  }
  while (cloud_blob != cloud_blob_prev);
  
  pcl::io::savePCDFile ("combined.pcd", *combined_cloud_ptr, true);
  ROS_INFO ("Saved %d data points to file.", (int)combined_cloud_ptr->points.size ());

  ROS_INFO ("Total number of PointCloud messages processed: %d", cnt);

  return (0);
}
/* ]--- */
