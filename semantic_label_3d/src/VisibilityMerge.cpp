#include <pcl_ros/io/bag_io.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>
//#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdint.h>
#include "pcl/ros/register_point_struct.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include "pcl/io/pcd_io.h"
#include <string>
#include <pcl_ros/io/bag_io.h>
//#include <pcl_ros/io/bag_io.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
//#include <Eigen/Core>
#include <tf/transform_listener.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_cloud.h"
typedef pcl::PointXYZRGBNormal PointT;
#include "includes/CombineUtils.h"
#include<set>
#include "pcl_visualization/pcl_visualizer.h"
using namespace std;
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;

bool detectOcclusion=true;
int
main(int argc, char** argv)
{
          sensor_msgs::PointCloud2 cloud_final_msg;
  sensor_msgs::PointCloud2 cloud_merged_all_points_msg;
    //  ros::init(argc, argv,"hi");
    pcl_visualization::PCLVisualizer *viewerPtr;
    bool interactive=false;
   int viewportCloud;
    int viewportCluster;
    int lastAcceptedIndex;
          ColorHandler::Ptr color_handler;

    if(argc>2&& argv[2][0]=='-'&& argv[2][1]=='i')
    {
        interactive=true;
        pcl_visualization::PCLVisualizer viewer("3D Viewer");
        viewer.createViewPort(0.0,0.0,0.5,1.0,viewportCloud);
        viewer.createViewPort(0.5,0.0,1.0,1.0,viewportCluster);

        viewerPtr=&viewer;
        std::cerr<<"starting in interactive mode" <<std::endl;
    }

    
    rosbag::Bag bag;
    std::cerr << "opening " << argv[1] << std::endl;
    bag.open(argv[1], rosbag::bagmode::Read);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());// cloud_transformed(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr  cloud_normal(new pcl::PointCloud<PointT > ());// cloud_transformed(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYGRGBCam> ());
    pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr final_cloud_backup(new pcl::PointCloud<pcl::PointXYGRGBCam> ());
    pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYGRGBCam> ());

    int tf_count = 0;
    int pcl_count = 0;
    int numOccluded=0;
    int numOcclRepudiated=0;
    bool check;

    std::vector<TransformG> transformsG;
    std::vector<pcl::PointCloud<PointT>::Ptr> pointClouds;
    std::vector<pcl::KdTreeFLANN<PointT>::Ptr> searchTrees;
    pcl::PCDWriter writer;
  std::vector<int> k_indices;
  std::vector<float> k_distances;

    rosbag::Bag reader;
    rosbag::View view;
    rosbag::View::iterator it;
    try
      {
	reader.open (argv[1], rosbag::bagmode::Read);
	view.addQuery (reader, rosbag::TopicQuery ("/rgbdslam/batch_clouds"));
	
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
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev, cloud_blob_temp;
    pcl::PointCloud<pcl::PointXYZRGB> inp_cloud;

    int rejectCount=0;
    pcl::PassThrough<pcl::PointXYZRGB> pass_;
    do
    {
        cloud_blob_prev = cloud_blob;
        if (it != view.end ())
	  {
	    cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
	    ++it;
	  }
        cloud_blob = cloud_blob_temp;
        ros::Time ptime = cloud_blob->header.stamp;

        if (cloud_blob_prev != cloud_blob)
        {
            pcl::fromROSMsg(*cloud_blob, inp_cloud);
            //ROS_INFO ("PointCloud with %d data points and frame %s (%f) received.", (int)cloud.points.size (), cloud.header.frame_id.c_str (), cloud.header.stamp.toSec ());
            pcl_count++;
        }

        //    rosbag::View view(bag, rosbag::TopicQuery("/rgbdslam/my_clouds"));


        rosbag::View view_tf(bag, rosbag::TopicQuery("/tf"), ptime - ros::Duration(0, 1), ptime + ros::Duration(0, 100000000));
        //std::cerr<<(view_tf.size())<<endl;
        std::cerr << ptime << std::endl;
        //        std::cerr<<"qid:"<<pcl_ptr->header.seq<<endl;;
        tf_count = 0;

        tf::Transform final_tft;

        BOOST_FOREACH(rosbag::MessageInstance const mtf, view_tf)
        {
            tf::tfMessageConstPtr tf_ptr = mtf.instantiate<tf::tfMessage > ();
            assert(tf_ptr != NULL);
            std::vector<geometry_msgs::TransformStamped> bt;
            tf_ptr->get_transforms_vec(bt);
            tf::Transform tft(getQuaternion(bt[0].transform.rotation), getVector3(bt[0].transform.translation));

            //  transG.print();
            if (ptime == bt[0].header.stamp)
            {
                tf_count++;
                std::cerr << "tf qid:" << bt[0].header.seq << std::endl;
                final_tft = tft;
            }
            assert(tf_count <= 1);
        }

        if (tf_count == 1)
        {
            TransformG transG(final_tft);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB > (inp_cloud));
            //  transformPointCloud(transG.transformMat,inp_cloud_ptr,cloud_transformed);
            // applyFilters(inp_cloud_ptr,cloud_filtered);
            std::cerr << "Origin" << inp_cloud.sensor_origin_[0] << "," << inp_cloud.sensor_origin_[1] << "," << inp_cloud.sensor_origin_[2] << "," << inp_cloud.sensor_origin_[3] << std::endl;
            transG.print();

            pass_.setInputCloud(inp_cloud_ptr);
            pass_.filter(*cloud_filtered);
            appendNormals(cloud_filtered,cloud_normal);
            //app

            if (pcl_count == 1)
            {
                lastAcceptedIndex=1;
                appendCamIndexAndDistance(cloud_normal,final_cloud,0,transG.getOrigin());
                transformsG.push_back(transG);
                pcl::PointCloud<PointT>::Ptr aCloud(new pcl::PointCloud<PointT > ());
                *aCloud=*cloud_normal;
                pointClouds.push_back(aCloud);
                pcl::KdTreeFLANN<PointT>::Ptr nnFinder(new pcl::KdTreeFLANN<PointT>);
                nnFinder->setInputCloud(aCloud);
                searchTrees.push_back(nnFinder);

            }
            else if (pcl_count -lastAcceptedIndex>=5)
            {
                *final_cloud_backup=*final_cloud;
                PointT cpoint;
                pcl::PointCloud<PointT>::Ptr aCloud(new pcl::PointCloud<PointT > ());
                *aCloud=*cloud_normal;
                pcl::KdTreeFLANN<PointT>::Ptr nnFinder(new pcl::KdTreeFLANN<PointT>);


                std::cerr<<" processing "<<pcl_count<<std::endl;
                for (unsigned int p = 0; p < cloud_normal->size(); p++)
                {
                    bool occluded=false;
                    cpoint = cloud_normal->points[p];
                    size_t c;
                        VectorG vpoint(cpoint.x,cpoint.y,cpoint.z);
                    if(!transG.isPointVisible(vpoint)) // if this point is not visible in it's own cam, ignore it
                        continue;
                    for (c = 0; c < transformsG.size(); c++)
                    {
                        TransformG ctrans=transformsG[c];
                        if((ctrans.isPointVisible(vpoint))) // is it already visibile in a prev camera? then it might be an outlier
                        {
                            if(!detectOcclusion)
                              break;
                            // is it also not occluded in the same camera in which it is visible? only then it will be an outlier
                            pcl::PointCloud<PointT>::Ptr apc=pointClouds[c];
                            pcl::KdTreeFLANN<PointT>::Ptr annFinder=searchTrees[c];
                            int lpt;
                            // if any point in the same frame occludes it, it wont be an outlier
                                VectorG cam2point=vpoint.subtract(ctrans.getOrigin());
                                double distance=cam2point.getNorm();
                                double radiusCyl=0.05;
                                cam2point.normalize();
                                int numPointsInBw=(int)(distance/radiusCyl); // floor because we dont wanna consider points very closet to the target
                                set<int> indices;
                                indices.clear();
                            for(lpt=2;lpt<numPointsInBw-1;lpt++) // -1 because because we dont wanna consider points very closet to the target ... they could be false occulusions
                            {
                                VectorG offset=cam2point.multiply(lpt*radiusCyl);
                                VectorG linePt = offset.add(ctrans.getOrigin());
                                int numNeighbors = annFinder->radiusSearch(linePt.getAsPoint(), radiusCyl, k_indices, k_distances, 20);
                                //apc->
                                for (int nn = 0; nn < numNeighbors; nn++)
                                {
                                        indices.insert(k_indices[nn]);
                                }
                                k_indices.clear();
                                k_distances.clear();

                            }

                            set<int>::iterator iter;
                            occluded=false;
                            for (iter = indices.begin(); iter != indices.end(); iter++)
                            {
                                VectorG ppcPointV(apc->points[*iter]);
                                double distanceLine = ppcPointV.computeDistanceSqrFromLine(ctrans.getOrigin(), vpoint);
                                if (distanceLine < (0.004 * 0.004) && ppcPointV.isInsideLineSegment(ctrans.getOrigin(), vpoint))
                                {
                                    
                                    occluded = true;
                                    break;
                                }
                            }

                            if(!occluded)
                            {
                                break; // reject this point, must present in prev camera and not occluded
                            }
                            else
                            {
                                VectorG cam2point=vpoint.subtract(transG.getOrigin());//change to the cam of test point to check if this occluded point is a duplicate
                                double distance=cam2point.getNorm();
                                double radiusCylFalseOcc=0.1; // more than radiusCyl to ensure more repudiations.. if u change this, also change numPointsToConsider ... the idea is that for repudiation, we only wanna consider points close to the target
                                int numPointsToConsider=1;
                                cam2point.normalize();
                                int numPointsInBw=(int)ceil(distance/radiusCylFalseOcc); // ceil because we dont wanna miss points near the target
                                set<int> indices;
                                indices.clear();
                                int startIndex=numPointsInBw-numPointsToConsider;
                                if(startIndex<1)
                                    startIndex=1;
                                for (lpt = startIndex; lpt <= numPointsInBw; lpt++)
                                {
                                    VectorG offset = cam2point.multiply(lpt * radiusCylFalseOcc);
                                    VectorG linePt = offset.add(transG.getOrigin());// change origin to current cam
                                    int numNeighbors = annFinder->radiusSearch(linePt.getAsPoint(), radiusCylFalseOcc, k_indices, k_distances, 20);
                                    //apc->

                                    for (int nn = 0; nn < numNeighbors; nn++)
                                    {
                                        indices.insert(k_indices[nn]);
                                    }
                                    k_indices.clear();
                                    k_distances.clear();

                                }

                                set<int>::iterator iter;
                                occluded = false;
                                for (iter = indices.begin(); iter != indices.end(); iter++)
                                {
                                    VectorG ppcPointV(apc->points[*iter]);
                                    double distanceLine = ppcPointV.computeDistanceSqrFromLine(ctrans.getOrigin(), vpoint);
                                    if (distanceLine < (0.003 * 0.003*distance) && ppcPointV.isInsideLineSegment(ctrans.getOrigin(), vpoint)) //0.3 more the value, more repudiation => less points added
                                    {


                                        //occlusion possible

                                        if(cosNormal(apc->points[*iter],cpoint)>0.90) // more the value, less repudiation => more occlusion =>more points added
                                        {
                                            occluded = true;
                                            break; // =>point wont be added
                                        }
                                    }
                                }
                                //visible but occluded by some point in same frame
                                if(occluded)
                                {
                                   // std::cerr<<"occlusion repudiation detected "<<numOcclRepudiated++ <<" pcl no:"<< pcl_count<<std::endl;
                                    break;
                                }
                                else
                                {
                                   // std::cerr<<"occlusion detected "<<numOccluded++ <<" pcl no:"<< pcl_count<<std::endl;
                                }
                            }
                        }
                    }
                    if(c==transformsG.size())
                    {
                        pcl::PointXYGRGBCam newPoint;
                        newPoint.x=cpoint.x;
                        newPoint.y=cpoint.y;
                        newPoint.z=cpoint.z;
                        newPoint.rgb=cpoint.rgb;
                        newPoint.normal_x=cpoint.normal_x;
                        newPoint.normal_y=cpoint.normal_y;
                        newPoint.normal_z=cpoint.normal_z;
                        newPoint.cameraIndex=transformsG.size();
                        newPoint.distance=VectorG(cpoint).subtract(transG.getOrigin()).getNorm();
                        final_cloud->points.push_back(newPoint);
                    }
                    else
                    {
                        rejectCount++;
                    }
                }
                if (interactive) 
                {
                    *temp_cloud=*final_cloud_backup;//+*cloud_normal;
                    pcl::toROSMsg(*final_cloud, cloud_final_msg);
                    pcl::toROSMsg(*temp_cloud, cloud_merged_all_points_msg);

                    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_final_msg));
                    viewerPtr->addPointCloud(*final_cloud, color_handler, "cloud", viewportCloud);
                    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_merged_all_points_msg));
                    viewerPtr->addPointCloud(*temp_cloud, color_handler, "cluster", viewportCluster);
                    viewerPtr->spin();

                }
                else
                {
                    lastAcceptedIndex=pcl_count;
                }
                transformsG.push_back(transG);
                nnFinder->setInputCloud(aCloud);
                pointClouds.push_back(aCloud);
                searchTrees.push_back(nnFinder);

            }
        }
        else
        {
            std::cerr << "no tf found";
        }



    }
    while (cloud_blob != cloud_blob_prev);

    std::cerr<<"nof rejected points "<<rejectCount;;
 //   applyFilters(final_cloud, cloud_filtered);
    string filename=string(argv[1]).append(".combined.pcd");
            writer.write<pcl::PointXYGRGBCam > (filename, *final_cloud, true);

    bag.close();


    //  ros::NodeHandle n;
    //Instantiate the kinect image listener
    /*  OpenNIListener kinect_listener(n,
    //  OpenNIListener kinect_listener(
                                     "/rgbdslam/batch_clouds",
                                     "/rgbdslam/my_clouds"
                                     );//,  "/camera/depth/image_raw");
 
       ros::spin();
     */

    //  while(1){
    //   sleep(20);

    //  }

}


