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
#include "pcl/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic_reconfigure/server.h>
#include <scene_processing/labelviewerConfig.h>


//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef pcl::PointXYZRGBCamSL PointT;

dynamic_reconfigure::Server < scene_processing::labelviewerConfig > *srv;
scene_processing::labelviewerConfig conf;
boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportCloud = 0;
int viewportCluster = 0;
std::vector<std::string> labels;


sensor_msgs::PointCloud2 cloud_blob;
sensor_msgs::PointCloud2 cloud_blob_filtered;
sensor_msgs::PointCloud2 cloud_blob_colored;

pcl::PointCloud<PointT> cloud;
pcl::PCDWriter writer;
ColorHandlerPtr color_handler;
std::map<int, std::set<int> > label_mapping;
pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_colored(new pcl::PointCloud<PointT > ());
bool doUpdate=false;

float
sqrG(float y) {
    return y*y;
}
//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;

void get_sorted_indices(pcl::PointCloud<PointT> &incloud, std::vector<int> &segmentindices, int size) {
    std::map<int, int> countmap;
    for (int i = 1; i <= size; i++) {
        countmap[i] = 0;
    }
    for (size_t i = 0; i < incloud.points.size(); ++i) {
        countmap[incloud.points[i].segment ] = countmap[incloud.points[i].segment ] + 1;
    }
    std::multimap<int, int> inverted_countmap;
    for (std::map<int, int>::iterator it = countmap.begin(); it != countmap.end(); it++)
        inverted_countmap.insert(std::pair<int, int>(it->second, it->first));
    for (std::multimap<int, int>::reverse_iterator rit = inverted_countmap.rbegin(); rit != inverted_countmap.rend(); rit++)
        segmentindices.push_back(rit->second);

}

void get_label_mapping(pcl::PointCloud<PointT> &incloud, std::map<int, std::set <int> > &label_mapping) {
    std::map <int, int> seg_label_mapping;
    for (size_t i = 0; i < incloud.points.size(); ++i) {
        seg_label_mapping[incloud.points[i].segment] = incloud.points[i].label;

    }
    for (std::map<int, int>::iterator it = seg_label_mapping.begin(); it != seg_label_mapping.end(); it++)
        label_mapping[it->second].insert(it->first);

}

void apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int label) {
    ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points = incloud.points;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].label == label) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            outcloud.points[i].rgb = 0.00005;
     
        }
    }
}


//   int spin=1;

void spinThread() {
    std::cerr << "thread started";
    while (true)
        viewer.spinOnce(1000, true);
}

void reconfig(scene_processing::labelviewerConfig & config, uint32_t level) {
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    bool found = false;
    int labelNum = 0;

    if (conf.showLabel) {
        conf.showLabel = false;
        doUpdate = true;
        std::string labelStr(conf.label);
        for (size_t li = 0; li < labels.size(); li++) {
            if (labelStr.compare(labels.at(li)) == 0) {
                labelNum = li+1;
                found = true;
                break;
            }
        }
    }
    if (found) {
        ROS_INFO("label found %d",labelNum);
        viewer.removePointCloud("labeled");
        apply_segment_filter(*cloud_ptr, *cloud_colored, labelNum);
        ROS_INFO("filetered sucessfully");
        pcl::toROSMsg (*cloud_colored,cloud_blob_filtered);
        color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered));
        viewer.addPointCloud(*cloud_colored, color_handler, "labeled", viewportCluster);
    }else
    {
        conf.message = "label not found!";
        doUpdate = true;
        viewer.removePointCloud("labeled");
        color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
        viewer.addPointCloud(*cloud_ptr, color_handler, "labeled", viewportCluster);
    }
    

}

/* ---[ */
int
main(int argc, char** argv) {

    ros::init(argc, argv, "labelviewer");
    bool groundSelected = false;
    bool editLabel = false;
    int targetLabel;
    if (argc > 2) {
        editLabel = true;
        targetLabel = atoi(argv[2]);
    }

    boost::numeric::ublas::matrix<double> outMat(4, 4);

    std::ifstream labelFile;
    std::string line;
    labelFile.open("/opt/ros/unstable/stacks/scene_processing/labels.txt");

    std::cerr << "you can only quit by pressing 9 when the prompt mentions... quitting in other ways will discard any newly added labels\n";
    if (labelFile.is_open()) {
        int count = 1;
        while (labelFile.good()) {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0)
                break;
            cout << "adding label " << line << " with value:" << count << endl;
            count++;
            labels.push_back(line);
        }
    } else {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }



    // read from file
    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), argv[1], pcl::getFieldsList(cloud_blob).c_str());

    // Convert to the templated message type
    pcl::fromROSMsg(cloud_blob, cloud);
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));


    // find the max segment number
    int max_segment_num = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (max_segment_num < cloud.points[i].segment) {
            max_segment_num = cloud.points[i].segment;
        }
    }

    std::vector<int> segmentIndices;
    // get_sorted_indices(*cloud_ptr, segmentIndices, max_segment_num);
    get_label_mapping(*cloud_ptr, label_mapping);

    // get the


    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewportCloud);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewportCluster);
    //for (int i = 1 ; i <= max_segment_num ; i++ ){
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
    viewer.addPointCloud(*cloud_ptr, color_handler, "cloud", viewportCloud);
    viewer.addPointCloud(*cloud_ptr, color_handler, "labeled", viewportCluster);
    viewer.spinOnce(5000, true);


    srv = new dynamic_reconfigure::Server < scene_processing::labelviewerConfig > (global_mutex);
    dynamic_reconfigure::Server < scene_processing::labelviewerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


    srv->setCallback(f);
    conf.done = false;

    bool isDone = false;
    //ROS_INFO ("Press q to quit.");
    while (!isDone) {
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
    cout << "normal kill";
    return (0);




    viewer.removePointCloud("labeled");
    viewer.removePointCloud("cloud");


    return (0);
}
/* ]--- */
