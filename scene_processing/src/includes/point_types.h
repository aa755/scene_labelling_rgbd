/* 
 * File:   point_types.h
 * Author: aa755
 *
 * Created on June 21, 2011, 12:22 AM
 */

#ifndef POINT_TYPES_H
#define	POINT_TYPES_H
 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 #include <pcl/io/pcd_io.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/kdtree/kdtree.h>
#include "pcl/kdtree/tree_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/impl/tree_types.hpp"
#include "pcl/kdtree/impl/organized_data.hpp"
#include "pcl/features/normal_3d.h"
#include "pcl/features/impl/normal_3d.hpp"

namespace pcl
{
    struct PointXYGRGBCam
    {
        PCL_ADD_POINT4D;
       float rgb;
        PCL_ADD_NORMAL4D;
       uint32_t cameraIndex;
       float distance;
             float curvature;

    };

    struct PointXYZRGBCamSL
    {
        PCL_ADD_POINT4D;
  union
  {
    struct
    {
      float rgb;
    };
    float data_c[4];
  };
       uint32_t cameraIndex;
       float distance;
       uint32_t segment;
       uint32_t label;
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYGRGBCam,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        )

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYZRGBCamSL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        (uint32_t, segment, segment)
        (uint32_t, label, label)
        )

//PCL_INSTANTIATE_initTree(pcl::PointXYZRGBCamSL)

#endif	/* POINT_TYPES_H */

