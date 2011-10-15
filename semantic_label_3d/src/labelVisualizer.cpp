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

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic_reconfigure/server.h>
#include <semantic_label_3d/labelviewerConfig.h>
#include "includes/color.cpp"

//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef pcl::PointXYZRGBCamSL PointT;
using namespace boost;
dynamic_reconfigure::Server < semantic_label_3d::labelviewerConfig > *srv;
semantic_label_3d::labelviewerConfig conf;
boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportOrig = 0;
int viewportPred = 0;
std::vector<std::string> labels;


sensor_msgs::PointCloud2 cloud_blob_orig;
sensor_msgs::PointCloud2 cloud_blob_pred;
sensor_msgs::PointCloud2 cloud_blob_filtered_orig;
sensor_msgs::PointCloud2 cloud_blob_filtered_pred;


pcl::PointCloud<PointT> cloud_orig;
pcl::PointCloud<PointT> cloud_pred;
pcl::PCDWriter writer;
ColorHandlerPtr color_handler_orig;
ColorHandlerPtr color_handler_pred;
std::map<int, std::set<int> > label_mapping_orig;
std::map<int, std::set<int> > label_mapping_pred;

pcl::PointCloud<PointT>::Ptr cloud_colored_pred(new pcl::PointCloud<PointT > ());
pcl::PointCloud<PointT>::Ptr cloud_colored_orig(new pcl::PointCloud<PointT > ());
bool doUpdate = false;

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

bool apply_label_filter(pcl::PointCloud<PointT> &incloud, int label, float color) {
    ROS_INFO("applying filter");
    bool changed = false;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].label == label) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            incloud.points[i].rgb = color;
            changed = true;
        }
    }
    return changed;
}

bool apply_label_filter(pcl::PointCloud<PointT> &incloud, vector<int> labels, float color) {
    ROS_INFO("applying filter");
    bool changed = false;

    for (size_t l = 0; l < labels.size(); l++) {
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].label == labels[l]) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            incloud.points[i].rgb = color;
            changed = true;
        }
    }
    }
    return changed;
}

bool apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    ROS_INFO("applying filter");
    bool changed = false;

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points = incloud.points;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            outcloud.points[i].rgb = 0.00001;
            changed = true;
        }
    }
    return changed;
}

//   int spin=1;

void spinThread() {
    std::cerr << "thread started";
    while (true)
        viewer.spinOnce(1000, true);
}

vector <string> getTokens(std::string str)
{
        char_separator<char> sep(",");
        tokenizer<char_separator<char> > tokens(str, sep);
       vector<string> out; 
        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(t);
        }
	return out;
}

void reconfig(semantic_label_3d::labelviewerConfig & config, uint32_t level) {
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
    pcl::PointCloud<PointT>::Ptr pred_cloud_ptr(new pcl::PointCloud<PointT > (cloud_pred));
    pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));
    bool c = false;

    int NUM_CLASSES_TO_SHOW=10;
    int labelNum = 0;

    if (conf.showLabel) {
        conf.showLabel = false;
        viewer.setBackgroundColor (1.0,1.0,1.0);
        doUpdate = true;
        string selLabels[NUM_CLASSES_TO_SHOW];
        selLabels[0]=conf.red_label;
        selLabels[1]=conf.green_label;
        selLabels[2]=conf.blue_label;
        selLabels[3]=conf.yellow_label;
        selLabels[4]=conf.cyan_label;
        selLabels[5]=conf.magenta_label;
        selLabels[6]=conf.dark_red_label;
        selLabels[7]=conf.dark_green_label;
        selLabels[8]=conf.dark_blue_label;
        selLabels[9]=conf.dark_yellow_label;

        ColorRGB *labelColors[NUM_CLASSES_TO_SHOW];
        labelColors[0]= new ColorRGB(1,0,0);
        labelColors[1]= new ColorRGB(0,1,0);
        labelColors[2]= new ColorRGB(0,0,1);
        labelColors[3]= new ColorRGB(1,1,0);
        labelColors[4]= new ColorRGB(0,1,1);
        labelColors[5]= new ColorRGB(1,0,1);
        labelColors[6]= new ColorRGB(0.5,0,0);
        labelColors[7]= new ColorRGB(0,0.5,0);
        labelColors[8]= new ColorRGB(0,0,0.5);
        labelColors[9]= new ColorRGB(0.5,0,0.5);

         *cloud_colored_orig=*orig_cloud_ptr;
         *cloud_colored_pred=*pred_cloud_ptr;
         int charCount=0.4;
        for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++) {
            //cout<<labelColors[color]->r*255.0<<labelColors[color]->g*255.0<<labelColors[color]->b*255.0<<endl;
            viewer.addText (selLabels[color],charCount*11,50,labelColors[color]->r,labelColors[color]->g,labelColors[color]->b);
            charCount=charCount+selLabels[color].size ()+0.9;
            cerr<<"showing label "<<selLabels[color];
        
            
        bool found = false;
        std::string labelStr(selLabels[color]);
        
        std::vector<std::string> selLabels=getTokens(labelStr);
        vector<int> labelNums;
        for (size_t lmi = 0; lmi < selLabels.size(); lmi++) {
            for (size_t li = 0; li < labels.size(); li++) {
                if (selLabels[lmi].compare(labels.at(li)) == 0) {
                    labelNums.push_back(li + 1);
                    found = true;
                    break;
                }
            }
        }
        
            if (found) {
                ROS_INFO("label found %d", labelNum);
                viewer.removePointCloud("orig");
                viewer.removePointCloud("pred");
                c = apply_label_filter(*cloud_colored_orig, labelNums,labelColors[color]->getFloatRep());
                ROS_INFO("filetered sucessfully");
                if (c) {
                        for (size_t lmi = 0; lmi < labelNums.size(); lmi++) {
                    std::cerr << "Orig cloud: List of segments with the label " << labelStr << " : ";
                    for (std::set<int>::iterator it = label_mapping_orig[labelNums[lmi]].begin(); it != label_mapping_orig[labelNums[lmi]].end(); it++) {
                        std::cerr << *it << " , ";
                    }
                    std::cerr << std::endl;
                        }
                    //conf.message_orig = "label found!";

                } else {
                    //conf.message_pred = "label not found!";
                }
                doUpdate = true;
                pcl::toROSMsg(*cloud_colored_orig, cloud_blob_filtered_orig);
                color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered_orig));
                viewer.addPointCloud(*cloud_colored_orig, color_handler_orig, "orig", viewportOrig);

                c = apply_label_filter(*cloud_colored_pred, labelNums,labelColors[color]->getFloatRep());
                ROS_INFO("filetered sucessfully");
                if (c) {
                        for (size_t lmi = 0; lmi < labelNums.size(); lmi++) {
                    std::cerr << "Orig cloud: List of segments with the label " << labelStr << " : ";
                    for (std::set<int>::iterator it = label_mapping_orig[labelNums[lmi]].begin(); it != label_mapping_orig[labelNums[lmi]].end(); it++) {
                        std::cerr << *it << " , ";
                    }
                    std::cerr << std::endl;
                        }
                    //conf.message_orig = "label found!";

                    //conf.message_pred = "label found!";
                } else {

                    //conf.message_pred = "label not found!";
                }
                doUpdate = true;
                pcl::toROSMsg(*cloud_colored_pred, cloud_blob_filtered_pred);
                color_handler_pred.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered_pred));
                viewer.addPointCloud(*cloud_colored_pred, color_handler_pred, "pred", viewportPred);

            }/* else {
                //conf.message_orig = "label not found!";
                //conf.message_pred = "label not found!";
                doUpdate = true;
                viewer.removePointCloud("orig");
                color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_orig));
                viewer.addPointCloud(*orig_cloud_ptr, color_handler_orig, "orig", viewportOrig);
                viewer.removePointCloud("pred");
                color_handler_pred.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_pred));
                viewer.addPointCloud(*pred_cloud_ptr, color_handler_pred, "pred", viewportPred);
            }*/
        }
    }
    if (conf.showSegment) {
        conf.showSegment = false;
        doUpdate = true;

            viewer.removePointCloud("orig");

            c = apply_segment_filter(*orig_cloud_ptr, *cloud_colored_orig, conf.segmentNum);


            pcl::toROSMsg(*cloud_colored_orig, cloud_blob_filtered_orig);
            color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered_orig));
            viewer.addPointCloud(*cloud_colored_orig, color_handler_orig, "orig", viewportOrig);


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
    labelFile.open("labels.txt");

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
    if (pcl::io::loadPCDFile(argv[1], cloud_blob_orig) == -1) {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob_orig.width * cloud_blob_orig.height), argv[1], pcl::getFieldsList(cloud_blob_orig).c_str());
    if (pcl::io::loadPCDFile(argv[2], cloud_blob_pred) == -1) {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob_pred.width * cloud_blob_pred.height), argv[2], pcl::getFieldsList(cloud_blob_pred).c_str());



    // Convert to the templated message type
    pcl::fromROSMsg(cloud_blob_orig, cloud_orig);
    pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));

    pcl::fromROSMsg(cloud_blob_pred, cloud_pred);
    pcl::PointCloud<PointT>::Ptr pred_cloud_ptr(new pcl::PointCloud<PointT > (cloud_pred));



    std::vector<int> segmentIndices;
    // get_sorted_indices(*cloud_ptr, segmentIndices, max_segment_num);
    get_label_mapping(*pred_cloud_ptr, label_mapping_pred);
    get_label_mapping(*orig_cloud_ptr, label_mapping_orig);

    


    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewportOrig);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewportPred);
    //for (int i = 1 ; i <= max_segment_num ; i++ ){
    color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_orig));

    color_handler_pred.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_pred));
    viewer.addPointCloud(*orig_cloud_ptr, color_handler_orig, "orig", viewportOrig);
    viewer.addPointCloud(*pred_cloud_ptr, color_handler_pred, "pred", viewportPred);
    //viewer.spinOnce(5000, true);


    srv = new dynamic_reconfigure::Server < semantic_label_3d::labelviewerConfig > (global_mutex);
    dynamic_reconfigure::Server < semantic_label_3d::labelviewerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


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

    string filename=string(argv[1]).append(".labelColored.pcd");
    writer.write<PointT > (filename, *cloud_colored_pred, true);
    cout << "normal kill";
    return (0);




    viewer.removePointCloud("orig");
    viewer.removePointCloud("pred");


    return (0);
}
/* ]--- */
