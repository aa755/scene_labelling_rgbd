/* 
 * File:   segmentation_new.cpp
 * Author: hema
 *
 * Created on September 18, 2011, 4:08 PM
 */

#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "includes/genericUtils.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"
//#include "descriptors_3d/all_descriptors.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "moveRobot.h"
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <point_cloud_mapping/geometry/nearest.h>
#include <pcl_ros/io/bag_io.h>
#include "HOG.cpp"
typedef pcl::PointXYZRGBCamSL PointT;
#include "includes/CombineUtils.h"
#include<boost/numeric/ublas/matrix.hpp>
#include<boost/numeric/ublas/io.hpp>
#include<boost/dynamic_bitset.hpp>
//#include<boost/numeric/bindings/traits/ublas_matrix.hpp>
//#include<boost/numeric/bindings/lapack/gels.hpp>
//#include <boost/numeric/bindings/traits/ublas_vector2.hpp>
//namespace ublas = boost::numeric::ublas;
//namespace lapack= boost::numeric::bindings::lapack;
#include "pcl_visualization/pcl_visualizer.h"
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
#include "time.h"
typedef pcl::KdTree<PointT> KdTree;
typedef pcl::KdTree<PointT>::Ptr KdTreePtr;
using namespace boost;
#include "includes/segmentAndLabel.h"
#include "openni_listener.h"
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;
using namespace octomap;
#include "wallDistance.h"
#include <pcl/io/pcd_io.h>
using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGBCamSL> cloud;
    assert(argc==2);
    pcl::io::loadPCDFile(argv[1],cloud);
    segmentInPlace(cloud,1);
    pcl::io::savePCDFileBinary(string(argv[1])+"_fine_seg.pcd",cloud);
    
    return 0;
}

