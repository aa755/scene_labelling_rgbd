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
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl_ros/io/bag_io.h>

#include "pcl_visualization/pcl_visualizer.h"
#include <dynamic_reconfigure/server.h>
#include <semantic_label_3d/pcmergerConfig.h>
#include "transformation.h"
typedef pcl::PointXYZRGBCamSL PointT;
#include "includes/CombineUtils.h"
//#include "includes/CombineUtils.h"


typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;


std::string fn;
dynamic_reconfigure::Server < semantic_label_3d::pcmergerConfig > *srv;
semantic_label_3d::pcmergerConfig conf;
boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportOrig = 0;
ColorHandlerPtr color_handler_new;
ColorHandlerPtr color_handler_prev;
ColorHandlerPtr color_handler_merged;



//sensor_msgs::PointCloud2 cloud_blob_new;
//sensor_msgs::PointCloud2 cloud_blob_prev;
sensor_msgs::PointCloud2 cloud_blob_merged;
sensor_msgs::PointCloud2ConstPtr cloud_blob_new, cloud_blob_prev, cloud_blob_temp;
sensor_msgs::PointCloud2 cloud_blobc_new;
sensor_msgs::PointCloud2 cloud_blobc_prev;
sensor_msgs::PointCloud2 cloud_blobc_mod;
sensor_msgs::PointCloud2 cloud_blobc_merged;


pcl::PointCloud<PointT> cloud_new;
pcl::PointCloud<PointT> cloud_prev;
pcl::PointCloud<PointT> cloud_merged;
pcl::PCDWriter writer;



pcl::PointCloud<PointT>::Ptr cloud_prev_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_new_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_ptr(new pcl::PointCloud<pcl::PointXYZRGB > ());
pcl::PointCloud<PointT>::Ptr cloud_mod_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_merged_ptr(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_merged_backup_ptr(new pcl::PointCloud<PointT > ());
bool doUpdate = false;
bool Merged = false;
bool ITpresent = false;
bool check;
rosbag::Bag reader;
rosbag::View view;
rosbag::View::iterator it;
int skipNum = 20;

semantic_label_3d::pcmergerConfig InitialTransformConfig;

bool noMoreUndo=false;

void updateUI() {
    pcl::PointCloud<PointT> cloud;


    //transform the new point cloud
    transformXYZYPR(*cloud_new_ptr, *cloud_mod_ptr, conf.x, conf.y, conf.z, conf.yaw/180.0*PI, conf.pitch/180.0*PI, conf.roll/180.0*PI);
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
void reconfig(semantic_label_3d::pcmergerConfig & config, uint32_t level) {
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
 //   pcl::PointCloud<PointT>::Ptr prev_cloud_ptr(new pcl::PointCloud<PointT > ());
  //  pcl::PointCloud<PointT>::Ptr new_cloud_ptr(new pcl::PointCloud<PointT > ());


    if(conf.add_pc) {
       //conf.add_pc = false;
        doUpdate = true;
        *cloud_new_ptr = *cloud_mod_ptr;
        *cloud_merged_backup_ptr=*cloud_merged_ptr;
        if(Merged)
        {
            *cloud_merged_ptr += *cloud_new_ptr;
            Matrix4f globalTrans=computeTransformXYZYPR(config.x, config.y, config.z, config.yaw/180.0*PI, config.pitch/180.0*PI, config.roll/180.0*PI);
            writeMatrixToFile(globalTrans);

        }
        else
            *cloud_merged_ptr = *cloud_new_ptr;
        //*cloud_merged_ptr =*cloud_new_ptr;
        
        viewer.removePointCloud("new");
        if(Merged)
            viewer.removePointCloud("merged");
        Merged = true;
        pcl::toROSMsg(*cloud_merged_ptr, cloud_blobc_merged);
        color_handler_merged.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_merged));
        viewer.addPointCloud(*cloud_merged_ptr, color_handler_merged, "merged", viewportOrig);
        ROS_INFO("displaying mergered pointcloud");

    }

    if(conf.setIT) {

        std::string transformsFileName=fn+".transforms.txt";
            transformFile.open(transformsFileName.data());

conf.setIT = false;
        doUpdate = true;
        InitialTransformConfig = conf;
        
            Matrix4f globalTrans=computeTransformXYZYPR(InitialTransformConfig.x, InitialTransformConfig.y, InitialTransformConfig.z, InitialTransformConfig.yaw/180.0*PI, InitialTransformConfig.pitch/180.0*PI, InitialTransformConfig.roll/180.0*PI);
            writeMatrixToFile(globalTrans);

        *cloud_new_ptr = *cloud_mod_ptr;
        ITpresent = true;
    }
    if(config.undo)
    {
        if(noMoreUndo)
        {
            conf.undo=false;
            doUpdate=true;
            return;
        }

        noMoreUndo=true;
        transformFile<<"endo"<<endl;
        *cloud_merged_ptr=*cloud_merged_backup_ptr;
        if(Merged)
            viewer.removePointCloud("merged");
        Merged = true;
        pcl::toROSMsg(*cloud_merged_ptr, cloud_blobc_merged);
        color_handler_merged.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_merged));
        viewer.addPointCloud(*cloud_merged_ptr, color_handler_merged, "merged", viewportOrig);
        ROS_INFO("undo:displaying mergered pointcloud");
        conf.undo=false;
        doUpdate=true;
  if (pcl::io::loadPCDFile (std::string("tempAppend.pcd"), cloud_blobc_new) == -1)
  {
    ROS_ERROR ("Couldn't read file ");
    return ;
  }
//  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1] ,pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blobc_new, *cloud_new_ptr);
//   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));

            if(ITpresent){
                cout<<"inside IT"<<endl;
                transformXYZYPR<PointT>(*cloud_new_ptr, *cloud_mod_ptr, InitialTransformConfig.x, InitialTransformConfig.y, InitialTransformConfig.z, InitialTransformConfig.yaw/180.0*PI, InitialTransformConfig.pitch/180.0*PI, InitialTransformConfig.roll/180.0*PI);
                *cloud_new_ptr = *cloud_mod_ptr;
                conf.pitch=0;
                conf.yaw=0;
                conf.roll=0;
            }
            //ROS_INFO("PointCloud with %d data points and frame %s (%f) received.", (int) cloud_new_ptr->points.size(), cloud_new_ptr->header.frame_id.c_str(), cloud_new_ptr->header.stamp.toSec());
            viewer.removePointCloud("new");
           // pcl::toROSMsg<PointT>(*cloud_new_ptr, cloud_blobc_new);
            color_handler_new.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_new));
            viewer.addPointCloud(*cloud_new_ptr, color_handler_new, "new", viewportOrig);
            ROS_INFO("undo:displaying new point cloud");
            conf.x=0;
            conf.y=0;
            conf.z=0;
            conf.yaw=0;
            conf.pitch=0;
            conf.roll=0;

        
    }
    if(conf.skip_pc || conf.add_pc) {
        noMoreUndo=false;
        conf.skip_pc = false;
        conf.add_pc =false;
        
        doUpdate = true;
        int count = 0;
        cloud_blob_prev = cloud_blob_new;
        if (it != view.end ())
          {
            cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
            ++it;
          }
        cloud_blob_new = cloud_blob_temp;
        cout<<"header"<<cloud_blob_new->header<<endl;
//        cloud_blob_new->
        ros::M_string::iterator iter;
        //for(iter=cloud_blob_new->__connection_header->begin ();iter!=cloud_blob_new->__connection_header->end ();iter++)
         // cout<<iter->first<<","<<iter->second<<endl;
        
        
        while(count < skipNum && cloud_blob_prev != cloud_blob_new)
        {
            cloud_blob_prev = cloud_blob_new;
	    if (it != view.end ())
	      {
		cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
		++it;
	      }
	    cloud_blob_new = cloud_blob_temp;
            count ++;
        }
        if (cloud_blob_prev != cloud_blob_new) {
            pcl::fromROSMsg(*cloud_blob_new, *cloud_temp_ptr);
            cloud_temp_ptr->header;
            cout<<"numPoints="<<cloud_temp_ptr->size ()<<endl;
            appendCamIndexAndDistance (cloud_temp_ptr,cloud_new_ptr,globalFrameCount,VectorG(0,0,0));
            globalFrameCount++;
            writer.write (std::string("tempAppend.pcd"),*cloud_new_ptr,true);
  if (pcl::io::loadPCDFile (std::string("tempAppend.pcd"), cloud_blobc_new) == -1)
  {
    ROS_ERROR ("Couldn't read temp file ");
    return ;
  }
//  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1] ,pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blobc_new, *cloud_new_ptr);
//   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
            
            if(ITpresent){
                cout<<"inside IT"<<endl;
                transformXYZYPR<PointT>(*cloud_new_ptr, *cloud_mod_ptr, InitialTransformConfig.x, InitialTransformConfig.y, InitialTransformConfig.z, InitialTransformConfig.yaw/180.0*PI, InitialTransformConfig.pitch/180.0*PI, InitialTransformConfig.roll/180.0*PI);
                *cloud_new_ptr = *cloud_mod_ptr;
                conf.pitch=0;
                conf.yaw=0;
                conf.roll=0;
            }
            //ROS_INFO("PointCloud with %d data points and frame %s (%f) received.", (int) cloud_new_ptr->points.size(), cloud_new_ptr->header.frame_id.c_str(), cloud_new_ptr->header.stamp.toSec());
            viewer.removePointCloud("new");
           // pcl::toROSMsg<PointT>(*cloud_new_ptr, cloud_blobc_new);
            color_handler_new.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_new));
            viewer.addPointCloud(*cloud_new_ptr, color_handler_new, "new", viewportOrig);
            ROS_INFO("displaying new point cloud");
            conf.x=0;
            conf.y=0;
            conf.z=0;
            conf.yaw=0;
            conf.pitch=0;
            conf.roll=0;
            
          /*  if(Merged){
                viewer.removePointCloud("merged");
                pcl::toROSMsg(*cloud_merged_ptr, cloud_blobc_merged);
                color_handler_merged.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blobc_merged));
                viewer.addPointCloud(*cloud_merged_ptr, color_handler_merged, "merged", viewportOrig);
            }*/
        }else {
            ROS_INFO("Finised reading all pointclouds! Select done to save.");
        }
    }
    
    if(conf.set_skip){
        conf.set_skip = false;
        doUpdate = true;
        skipNum = (conf.skipNum);
    }
    if(conf.update)
    {
        conf.update = false;
        doUpdate = true;
        updateUI();
    }
}




/* ---[ */
int
main(int argc, char** argv) {
       fn = "transformed_"  + std::string(argv[1])+".pcd";

  ros::init(argc, argv, "pcmerger");
  char *topic="/camera/rgb/points";
  if(argc>2)
    topic=argv[2];

    try
      {
        reader.open (argv[1], rosbag::bagmode::Read);
        view.addQuery (reader, rosbag::TopicQuery (topic));

        if (view.size () == 0)
          check = false;
        else
          it = view.begin ();
      }
    catch (rosbag::BagException &e)
      {
        check = false;
      }
    check = true;
  if (!check)
  {
    cout <<"Couldn't read bag file on topic" <<(topic);
    return (-1);
  }
//  skipNum = atoi(argv[2]);


   viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewportOrig);
   viewer.addCoordinateSystem (1);

    //viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewportPred);
    //for (int i = 1 ; i <= max_segment_num ; i++ ){
    srv = new dynamic_reconfigure::Server < semantic_label_3d::pcmergerConfig > (global_mutex);
    dynamic_reconfigure::Server < semantic_label_3d::pcmergerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


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
      writer.write ( fn,*cloud_merged_ptr, true);
    transformFile.close();

    

   // viewer.removePointCloud("pred");


    return (0);
}
/* ]--- */
