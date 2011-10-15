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
#include <dynamic_reconfigure/server.h>
#include <semantic_label_3d/labelingConfig.h>

#include <pcl/filters/passthrough.h>



//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;

dynamic_reconfigure::Server < semantic_label_3d::labelingConfig > *srv;
semantic_label_3d::labelingConfig conf;

typedef pcl::PointXYZRGBCamSL PointT;
float
sqrG(float y)
{
    return y*y;
}
//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;
class VectorG
{
public:
    double v[3];

    VectorG()
    {
    }

    VectorG(double unitX, double unitY, double unitZ)
    {
        v[0] = unitX;
        v[1] = unitY;
        v[2] = unitZ;


    }

    bool isUnitVector()
    {
        return (isZero(getNormSqr()-1));
        
    }
    
    VectorG(PointT p)
    {
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;


    }
    /**
     * compute the distance from the line joining p1 and p2
     */
    double computeDistanceSqrFromLine(VectorG p1,VectorG p2)
    {
        VectorG t1=p1.subtract(*this);

        VectorG t2=p2.subtract(p1);

        return (t1.getNormSqr()*t2.getNormSqr()-sqrG(t1.dotProduct(t2)))/t2.getNormSqr();
    }

    /**
     *true iff it lies in the cylinder of infinite radius defined by p1 and p2
     * the line segment (p1,p2) is the axis of this cylinder
     */
    bool isInsideLineSegment(VectorG p1,VectorG p2)
    {
        VectorG lineSeg=p1.subtract(p2);
        double lengthSqr=lineSeg.getNormSqr();
        VectorG p1Seg=subtract(p1);
        VectorG p2Seg=subtract(p2);

        if(p1Seg.getNormSqr()<=lengthSqr&&p2Seg.getNormSqr()<=lengthSqr)
            return true;
        else
            return false;


    }

    void
    normalize()
    {
        double norm = getNorm();
        for (int i = 0; i < 3; i++)
            v[i] = v[i] / norm;
    }

    PointT getAsPoint()
    {
        PointT p;
        p.x=v[0];
        p.y=v[1];
        p.z=v[2];
        return p;
    }
    
    double
    getNorm()
    {
        return sqrt(getNormSqr());
    }

    double
    getNormSqr()
    {
        return (sqrG(v[0]) + sqrG(v[1]) + sqrG(v[2]));
    }

    double
    dotProduct(VectorG v2g)
    {
        double sum = 0;
        for (int i = 0; i < 3; i++)
            sum = sum + v[i] * v2g.v[i];
        return sum;
    }

    
    static bool isZero(double d)
    {
         std::cerr<<"value is:"<<d<<endl;
        return (fabs(d)<0.001);
    }
    //(a2b3 − a3b2, a3b1 − a1b3, a1b2 − a2b1)
    VectorG
    crossProduct(VectorG v2g)
    {
         VectorG ret(v[2-1]*v2g.v[3-1]-v[3-1]*v2g.v[2-1],  v[3-1]*v2g.v[1-1]-v[1-1]*v2g.v[3-1],  v[0]*v2g.v[2-1]-v[2-1]*v2g.v[1-1]);
         std::cerr<<"nsaorm is"<<ret.getNorm()<<endl;
        assert(isZero(ret.getNorm()-1));
        assert(isZero(ret.dotProduct(*this)));
        assert(isZero(ret.dotProduct(v2g)));
        return ret;
    }
    
    VectorG
    multiply(double scalar)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = scalar*v[i];
        return out;
    }

    VectorG
    subtract(VectorG v2g)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = v[i] - v2g.v[i];
        return out;

    }


    VectorG
    add(VectorG v2g)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = v[i] + v2g.v[i];
        return out;

    }

    float
    eucliedianDistance(VectorG v2g)
    {
        float sum=0;
        for (int i = 0; i < 3; i++)
            sum=sum + sqrG(v[i] - v2g.v[i]);
        return sqrt(sum);

    }

};

void transformPointCloud(boost::numeric::ublas::matrix<double> &transform, pcl::PointCloud<PointT>::Ptr in,
                    pcl::PointCloud<PointT>::Ptr out)
{

    boost::numeric::ublas::matrix<double> matIn(4, 1);
    *out = *in;

    for (size_t i = 0; i < in->points.size(); ++i)
    {
        double * matrixPtr = matIn.data().begin();

        matrixPtr[0] = in->points[i].x;
        matrixPtr[1] = in->points[i].y;
        matrixPtr[2] = in->points[i].z;
        matrixPtr[3] = 1;
        boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);
        matrixPtr = matOut.data().begin();

        out->points[i].x = matrixPtr[0];
        out->points[i].y = matrixPtr[1];
        out->points[i].z = matrixPtr[2];
    }
}
std::map<int, VectorG> centroids;
void get_sorted_indices (pcl::PointCloud<PointT> &incloud , std::vector<int> &segmentindices , int size)
{
  
   std::map<int,int> countmap;
   for (int i = 1; i <=size; i++)
   {
     countmap[i] = 0;
     centroids[i]=VectorG(0,0,0);
   }
   for (size_t i = 0; i < incloud.points.size (); ++i)
   {
     countmap[incloud.points[i].segment ] = countmap[incloud.points[i].segment ] + 1;
     centroids[incloud.points[i].segment]=centroids[incloud.points[i].segment].add (VectorG(incloud.points[i]));
   }
   for (int i = 1; i <=size; i++)
   {
     centroids[i]=centroids[i].multiply (1.0/countmap[i]);
   }
   
   std::multimap<int,int> inverted_countmap ;
   for ( std::map<int,int>::iterator it=countmap.begin() ; it != countmap.end(); it++ )
     inverted_countmap.insert (std::pair<int,int>(it->second ,it->first));
   for ( std::multimap<int,int>::reverse_iterator rit=inverted_countmap.rbegin(); rit != inverted_countmap.rend(); rit ++)
     segmentindices.push_back(rit->second);
     
}

void apply_segment_filter ( pcl::PointCloud<PointT> &incloud ,  pcl::PointCloud<PointT> &outcloud, pcl::PointCloud<PointT> &segment_cloud , uint32_t segment)
{
  //size_t
   ROS_INFO ("applying filter");
   outcloud.points.erase(outcloud.points.begin(),outcloud.points.end());
   segment_cloud.points.erase(segment_cloud.points.begin(),segment_cloud.points.end());
   outcloud.header.frame_id = incloud.header.frame_id;
   outcloud.points = incloud.points;
   segment_cloud.header.frame_id = incloud.header.frame_id;
   segment_cloud.points.resize ( incloud.points.size() );
   int j = 1;
   for (size_t i = 0; i < incloud.points.size (); ++i)
   {

     if(incloud.points[i].segment == segment) {
       segment_cloud.points[j].x = incloud.points[i].x;
       segment_cloud.points[j].y = incloud.points[i].y;
       segment_cloud.points[j].z = incloud.points[i].z;
       segment_cloud.points[j].rgb = incloud.points[i].rgb;
       segment_cloud.points[j].segment = incloud.points[i].segment;
       segment_cloud.points[j].label = incloud.points[i].label;
  //     std::cerr<<segment_cloud.points[j].label<<",";
       outcloud.points[i].rgb = 0.00005 ;
       j++;
     }
  }
   segment_cloud.points.resize ( j );
}

   pcl_visualization::PCLVisualizer viewer("3D Viewer");
//   int spin=1;
void spinThread()
{
    std::cerr<<"thread started";
    while(true)
        viewer.spinOnce(1000,true);
}

    int viewportCloud = 0;
    int viewportCluster = 0;

boost::recursive_mutex global_mutex;
  bool doUpdate=false;
    std::vector<std::string> labels;//(initLabels);
  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2 cloud_blob_filtered;
  sensor_msgs::PointCloud2 cloud_blob_colored;

  pcl::PointCloud<PointT> cloud;
 //ColorHandlerPtr color_handler;
  pcl::PCDWriter writer;
  ColorHandlerPtr color_handler;

  pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr  cloud_colored (new pcl::PointCloud<PointT> ());
  std::map<int,int> label_mapping; 
  std::vector<int> segmentIndices; 
  std::vector<int>::iterator seg_iter;
// * Dynamic reconfigure callback */

  bool processPointCloud(int seg_no)
  {
    
    if(centroids[seg_no].v[0]>=conf.maxx ||centroids[seg_no].v[0]<conf.minx ||centroids[seg_no].v[1]>=conf.maxy ||centroids[seg_no].v[1]<conf.miny ||centroids[seg_no].v[2]>=conf.maxz ||centroids[seg_no].v[2]<conf.minz)
    return false;
      apply_segment_filter( cloud, *cloud_colored , *cloud_filtered, seg_no);
      if(cloud_filtered->size ()<conf.minSegSize)
        return false;
/*           for (size_t i = 1; i < cloud_filtered->points.size(); ++i)
  {
                   cout<<cloud_filtered->points[i].x<<endl;
                   cout<<cloud_filtered->points[i].y<<endl;
                   cout<<cloud_filtered->points[i].z<<endl;
               if(!((cloud_filtered->points[i].x<conf.maxx&&cloud_filtered->points[i].x>=conf.minx)&&(cloud_filtered->points[i].y<conf.maxy&&cloud_filtered->points[i].y>=conf.miny)&&(cloud_filtered->points[i].z<conf.maxz&&cloud_filtered->points[i].z>=conf.minz)))
                 {
                        return false;
                 }
                 
  }*/

 //     cout<<"passed range test\n";
      
    size_t curLabel=cloud_filtered->points[1].label;
    
    if(curLabel==0&&label_mapping[seg_no]==0)
    {
        cout<<"not assigned a label yet\n";
    }
    else if(curLabel>0 && conf.skip_labeled&&labels.at(curLabel-1).compare (conf.no_skip_label)!=0)
      return false;
    else
    {
        if(curLabel==0)
          curLabel=label_mapping[seg_no];
        assert(curLabel>0&&curLabel<=labels.size());
        cout<<"segment "<<seg_no<<" had label :"<<labels.at(curLabel-1)<<endl;
    }

       ROS_INFO ("CLuster number %d",seg_no);
          viewer.removePointCloud("cluster",viewportCluster);
            viewer.removePointCloud("cloud",viewportCloud);

    pcl::toROSMsg (*cloud_filtered,cloud_blob_filtered);
    pcl::toROSMsg (*cloud_colored,cloud_blob_colored);

    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_colored));
    viewer.addPointCloud(*cloud_colored,color_handler,"cloud",viewportCloud);
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_filtered));
    viewer.addPointCloud(*cloud_filtered,color_handler ,"cluster",viewportCluster);
    return true;
  
  }
  std::string fn;
  
  void savePCDAndLabels()
  {
  std::vector<pcl::PointCloud<PointT> > clusters2;
  pcl::PointCloud<PointT> labeled_cloud;
  //pcl::PointCloud<PointT>::Ptr labeled_transformed_cloud(new pcl::PointCloud<PointT>());
  //pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
  //getClustersFromPointCloud2(*cloud_filtered, clusters, clusters2,combined_cloud);
  labeled_cloud.header = cloud.header;
  labeled_cloud.points = cloud.points;
  for (size_t i = 0; i< labeled_cloud.points.size(); i++)
  {
      int newLabel=label_mapping[labeled_cloud.points[i].segment];
      if(newLabel>0)
        labeled_cloud.points[i].label = newLabel;
  }
  
    std::ofstream labelFileOut;
    labelFileOut.open("/opt/ros/unstable/stacks/semantic_label_3d/labels.txt");
                 for(size_t li=0;li<labels.size();li++)
                 {
                     labelFileOut<<labels.at(li)<<endl;
                 }
    labelFileOut.close();
  //pcl::PointCloud<PointT>::Ptr labeled_cloud_ptr(new pcl::PointCloud<PointT> (labeled_cloud));

  writer.write ( fn,labeled_cloud, true);
    
  }

  void showClippedPointCloud()
  {
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT > (cloud));
          pcl::PointCloud<PointT>::Ptr temp1(new pcl::PointCloud<PointT> ());
          pcl::PointCloud<PointT>::Ptr temp2(new pcl::PointCloud<PointT> ());
          pcl::PointCloud<PointT>::Ptr clippedCloud(new pcl::PointCloud<PointT> ());
          pcl::PassThrough<PointT> pass;
          cout<<"original cloud has "<<cloud_ptr->size ()<<endl;
          pass.setInputCloud (cloud_ptr);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (conf.minz, conf.maxz);
          pass.filter (*temp1);
          cout<<"clipped x cloud has "<<temp1->size ()<<endl;

          pass.setInputCloud (temp1);
          pass.setFilterFieldName ("y");
          pass.setFilterLimits (conf.miny, conf.maxy);
          pass.filter (*temp2);
          cout<<"clipped y cloud has "<<temp2->size ()<<endl;
  
          pass.setInputCloud (temp2);
          pass.setFilterFieldName ("x");
          pass.setFilterLimits (conf.minx, conf.maxx);
          pass.filter (*clippedCloud);
          cout<<"clipped cloud has "<<clippedCloud->size ()<<endl;
          
          viewer.removePointCloud("cluster",viewportCluster);
            viewer.removePointCloud("cloud",viewportCloud);
          pcl::toROSMsg (*clippedCloud,cloud_blob_filtered);
        //pcl::toROSMsg (*cloud_colored,cloud_blob_colored);

    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob));
    viewer.addPointCloud(cloud,color_handler,"cloud",viewportCloud);
    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_blob_filtered));
    viewer.addPointCloud(*clippedCloud,color_handler ,"cluster",viewportCluster);
  //        viewer.removeCoordinateSystem ();
          
  }
  
void nextPointCloud()
{
            seg_iter++;
            while(seg_iter<segmentIndices.end () && !processPointCloud (*seg_iter))
                seg_iter++;
            if(seg_iter==segmentIndices.end ())
              {
                conf.label="finished labelling all. choose done";
                        doUpdate=true;
              }

                      
}
bool labellingMode=false; //clippingMode=(!labelingMode)
void reconfig(semantic_label_3d::labelingConfig & config, uint32_t level)
{
  conf=config;
  boost::recursive_mutex::scoped_lock lock(global_mutex);
  if((!labellingMode)&&(!conf.show_clipped)) // just started labeling => reset
    {
         seg_iter=segmentIndices.begin() ; 
            while(seg_iter<segmentIndices.end () && !processPointCloud (*seg_iter))
                seg_iter++;
//          viewer.removeCoordinateSystem ();
         labellingMode=true;
         return;
    }
  
  labellingMode=!conf.show_clipped;
  
  if(labellingMode)
    {
  if(config.accept_label)
    {
      std::string labelStr=config.label;
         cout <<"selected label:"<< labelStr << endl;
         bool found=false;
         for(size_t li=0;li<labels.size();li++)
         {
             if(labelStr.compare(labels.at(li))==0)
             {
                label_mapping[*seg_iter] = li+1;
                cout <<"label numerical alias:"<< li+1 << endl;

                found=true;
                break;
             }
         }
         if(found)
                nextPointCloud ();
         else
           std::cerr<<"suspected typo. use new label checkbox to add a new model"<<endl;
         
         conf.accept_label=false;
         doUpdate=true;
    }
  else if(config.new_label)
    {
      
      std::string labelStr=config.label;
                    cout<<"added new label:"<<labelStr<<endl;
                  labels.push_back(labelStr);
                label_mapping[*seg_iter] = labels.size();
                  //the next iteration will match and exit... no need to set done now
                  cout<<"new set of labels: \n";
                 for(size_t li=0;li<labels.size();li++)
                 {
                     cout<<labels.at(li)<<endl;
                 }

                  nextPointCloud ();
         conf.new_label=false;
         doUpdate=true;
    }
    }
  else 
    {
      showClippedPointCloud ();
    }
  

}


/* ---[ */
int
  main (int argc, char** argv)
{
    ros::init(argc, argv, "labelling");
    fn = "labeled_"  + std::string(argv[1]);

//    bool groundSelected=false;
    bool editLabel=false;
    int targetLabel;
    if(argc>2)
    {
        editLabel=true;
        targetLabel=atoi(argv[2]);
    }
        
            boost::numeric::ublas::matrix<double> outMat(4, 4);
    std::ifstream labelFile;
    std::string line;
    labelFile.open("/opt/ros/unstable/stacks/semantic_label_3d/labels.txt");

    std::cerr<<"you can only quit by pressing 9 when the prompt mentions... quitting in other ways will discard any newly added labels\n";
    if(labelFile.is_open())
    {
        int count=1;
        while(labelFile.good())
        {
            getline(labelFile,line);//each line is a label
            if(line.size()==0)
                break;
            cout<<"adding label "<<line<<" with value:"<<count<<endl;
            count++;
            labels.push_back(line);
        }
    }
    else
    {
        cout<<"could not open label file...exiting\n";
        exit(-1);
    }

 
 // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file ");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1] ,pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blob, cloud);
   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));

   pcl::PassThrough<PointT> pass;
   pass.setInputCloud (cloud_ptr);
   pass.filter(*cloud_ptr);
   //cout << "Size of cloud after filtering : " << cloud_ptr->points.size() << " , " << cloud.points.size() << endl; 
   cloud = *cloud_ptr;
   cout << "Size of cloud after filtering : " << cloud_ptr->points.size() << " , " << cloud.points.size() << endl; 
   
   
  // find the max segment number 
  size_t max_segment_num = 0;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
     if (max_segment_num < cloud.points[i].segment) {max_segment_num = cloud.points[i].segment;}    
  }
  get_sorted_indices ( *cloud_ptr , segmentIndices , max_segment_num);

  // get the 

    viewer.createViewPort(0.0,0.0,0.5,1.0,viewportCloud);
    viewer.createViewPort(0.5,0.0,1.0,1.0,viewportCluster);
 
      viewer.addCoordinateSystem (1);

    labellingMode=false;
    showClippedPointCloud ();
      viewer.spinOnce(1000,true);
  srv = new dynamic_reconfigure::Server < semantic_label_3d::labelingConfig > (global_mutex);
  dynamic_reconfigure::Server < semantic_label_3d::labelingConfig >::CallbackType f =
    boost::bind(&reconfig, _1, _2);
  
  srv->setCallback(f);
  conf.done=false;

  bool isDone=false;
  //ROS_INFO ("Press q to quit.");
  while (!isDone) {
    viewer.spinOnce ();
    ros::spinOnce();
    if(conf.done)
      {
        conf.done=false;
              srv->updateConfig(conf);
              savePCDAndLabels ();
      break;
      }
    if (doUpdate) {
      doUpdate = false;
      srv->updateConfig(conf);
    }
  }

  cout<<"normal kill";
  return (0);
}
/* ]--- */
