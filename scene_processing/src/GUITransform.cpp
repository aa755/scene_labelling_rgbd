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

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>



#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/point_types.h>

#include <pcl_ros/io/bag_io.h>

#include "pcl_visualization/pcl_visualizer.h"
#include <dynamic_reconfigure/server.h>
#include <scene_processing/pcmergerConfig.h>
#include "transformation.h"
typedef pcl::PointXYZRGBCamSL PointT;
#include "CombineUtils.h"
//#include "CombineUtils.h"


typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;


std::string fn;
dynamic_reconfigure::Server < scene_processing::pcmergerConfig > *srv;
scene_processing::pcmergerConfig conf;
boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportOrig = 0;
ColorHandlerPtr color_handler_new;
ColorHandlerPtr color_handler_prev;
ColorHandlerPtr color_handler_merged;



//sensor_msgs::PointCloud2 cloud_blob_new;
//sensor_msgs::PointCloud2 cloud_blob_prev;
sensor_msgs::PointCloud2 cloud_blob_merged;
sensor_msgs::PointCloud2ConstPtr cloud_blob_new, cloud_blob_prev;
sensor_msgs::PointCloud2 cloud_blobc_new;
sensor_msgs::PointCloud2 cloud_blobc_prev;
sensor_msgs::PointCloud2 cloud_blobc_mod;
sensor_msgs::PointCloud2 cloud_blobc_merged;


pcl::PointCloud<PointT> cloud_new;
pcl::PointCloud<PointT> cloud_prev;
pcl::PointCloud<PointT> cloud_merged;
pcl::PCDWriter writer;



pcl::PointCloud<PointT>::Ptr cloud_prev_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_ptr(new pcl::PointCloud<pcl::PointXYZRGB > ());
pcl::PointCloud<PointT>::Ptr cloud_mod_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_merged_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_merged_backup_ptr(new pcl::PointCloud<PointT > ());
bool doUpdate = false;
bool Merged = false;
bool ITpresent = false;
pcl_ros::BAGReader reader;
int skipNum = 20;

scene_processing::pcmergerConfig InitialTransformConfig;

bool noMoreUndo=false;

void updateUI() {
    pcl::PointCloud<PointT> cloud;


    //transform the new point cloud
    transformXYZYPR(*cloud_ptr, *cloud_mod_ptr, conf.x, conf.y, conf.z, conf.yaw/180.0*PI, conf.pitch/180.0*PI, conf.roll/180.0*PI);
    viewer.removePointCloud("new");
    pcl::toROSMsg(*cloud_mod_ptr, cloud_blobc_mod);
    color_handler_new.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_mod));
    viewer.addPointCloud(*cloud_mod_ptr, color_handler_new, "new", viewportOrig);
    
   // viewer.spinOnce();
}
    std::ofstream transformFile;

    void writeMatrixToFile(Matrix4f globalTrans)
    {
                 for(size_t li=0;li<16;li++)
                 {
                     transformFile<<globalTrans.data()[li]<<" ";
                     if(li%4==3)
                         transformFile<<endl;

                 }
    }
    int globalFrameCount=0;
void reconfig(scene_processing::pcmergerConfig & config, uint32_t level) {
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
 //   pcl::PointCloud<PointT>::Ptr prev_cloud_ptr(new pcl::PointCloud<PointT > ());
  //  pcl::PointCloud<PointT>::Ptr new_cloud_ptr(new pcl::PointCloud<PointT > ());



    if(conf.setIT) {
        conf.setIT=false;
        doUpdate=true;
        updateUI();
        double sum=0;
        double sqrsum=0;
        double zval;
        int countGround=0;
        for(int i=0;i<cloud_mod_ptr->size ();++i)
          {
            if(cloud_mod_ptr->points[i].label==2)
              {
                zval=cloud_mod_ptr->points[i].z;
                sum+=zval;
                sqrsum+=zval;
                countGround++;
              }
          }
        double meanSqrZ=sqrsum/countGround;
        double meanZ=sum/countGround;
        double stdDev=sqrt (meanSqrZ-meanZ);
        assert(stdDev<0.2);
        cout<<"std dev of z(ground points)"<<stdDev<<endl;
        conf.z=-meanZ;
        
        

    }
}

double mean (double * array, double N)
{
double sum = 0 ;
for (int i = 0; i < N; i++)
sum = sum + array [i];
return sum/N;
} // function calculating mean


double std_dev (double * array, double N)
{
double sum = 0;
double STD_DEV = 0; // returning zero's

for (int i = 0; i < N; i++)
{
sum = sum + array [i];
STD_DEV = STD_DEV + pow(array [i], 2);
}
return sqrt ((STD_DEV/N) - (pow(sum/N,2)));
} // function calculating standard deviation



/* ---[ */
int
main(int argc, char** argv) {
    ros::init(argc, argv, "transform");

       fn = "transformed_"  + std::string(argv[1]);
   viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewportOrig);
   viewer.addCoordinateSystem (1);
     sensor_msgs::PointCloud2 cloud_blob;

  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
    {
      ROS_ERROR ("Couldn't read file ");
      return (-1);
    }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), argv[1], pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
  pcl::fromROSMsg (cloud_blob, *cloud_ptr);

    //viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewportPred);
    //for (int i = 1 ; i <= max_segment_num ; i++ ){
    srv = new dynamic_reconfigure::Server < scene_processing::pcmergerConfig > (global_mutex);
    dynamic_reconfigure::Server < scene_processing::pcmergerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


    srv->setCallback(f);
    conf.done = false;
    while (true) {
        viewer.spinOnce();
        ros::spinOnce();
        if (conf.done) {
            conf.done = false;
            srv->updateConfig(conf);
            //savePCDAndLabels ();
            break;
        }
        if (doUpdate) {
            doUpdate = false;
            srv->updateConfig(conf);
        }
    }
    
    pcl::PCDWriter writer;
      writer.write ( fn,*cloud_mod_ptr, true);
    transformFile.close();

    

   // viewer.removePointCloud("pred");


    return (0);
}
/* ]--- */
