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

namespace pcl {

    struct PointXYGRGBCam {
        PCL_ADD_POINT4D;
        float rgb;
        PCL_ADD_NORMAL4D;
        uint32_t cameraIndex;
        float distance;
        float curvature;

    };
    
    struct PointPLY {
        PCL_ADD_POINT4D;
        uint32_t r;
        uint32_t g;
        uint32_t b;

    };

    struct PointXYInt {
        int x;
        int y;
        int z;
        int segment;
        int label;
    };

    struct PointXYZRGBCamSL {
        PCL_ADD_POINT4D;

        union {

            struct {
                float rgb;
            };
            float data_c[4];
        };
        uint32_t cameraIndex;
        float distance;
        uint32_t segment;
        uint32_t label;

        //      inline PointXYZRGBCamSL (float _x, float _y, float _z) { x = _x; y = _y; z = _z; data[3] = 1.0f; }

        void clone(const struct PointXYZRGB & rhs) {
            for (int i = 0; i < 4; i++)
                data[i] = rhs.data[i];
            rgb = rhs.rgb;
            segment = 0;
            label = 0;
            cameraIndex = 0;
            distance = 0.0;

        }
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
        pcl::PointXYInt,
        (int, x, x)
        (int, y, y)
        (int, z, z)
        (int, segment, segment)
        (int, label, label)
        )
        
POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointPLY,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (int, r, r)
        (int, g, g)
        (int, b, b)
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

