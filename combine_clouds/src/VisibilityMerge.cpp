#include "CombineUtils.h"
#include<set>
using namespace std;

int
main(int argc, char** argv)
{
    //  ros::init(argc, argv,"hi");

    rosbag::Bag bag;
    std::cerr << "opening " << argv[1] << std::endl;
    bag.open(argv[1], rosbag::bagmode::Read);
    pcl::PointCloud<PointT>::Ptr  cloud_filtered(new pcl::PointCloud<PointT > ());// cloud_transformed(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYGRGBCam> ());

    int tf_count = 0;
    int pcl_count = 0;
    int numOccluded=0;
    int numOcclRepudiated=0;

    std::vector<TransformG> transformsG;
    std::vector<pcl::PointCloud<PointT>::Ptr> pointClouds;
    std::vector<pcl::KdTreeFLANN<PointT>::Ptr> searchTrees;
  std::vector<int> k_indices;
  std::vector<float> k_distances;

    pcl_ros::BAGReader reader;
    if (!reader.open(argv[1], "/rgbdslam/my_clouds"))
    {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
    pcl::PointCloud<PointT> inp_cloud;

    int rejectCount=0;
    pcl::PassThrough<PointT> pass_;
    do
    {
        cloud_blob_prev = cloud_blob;
        cloud_blob = reader.getNextCloud();
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
            pcl::PointCloud<PointT>::Ptr inp_cloud_ptr(new pcl::PointCloud<PointT > (inp_cloud));
            //  transformPointCloud(transG.transformMat,inp_cloud_ptr,cloud_transformed);
            // applyFilters(inp_cloud_ptr,cloud_filtered);
            std::cerr << "Origin" << inp_cloud.sensor_origin_[0] << "," << inp_cloud.sensor_origin_[1] << "," << inp_cloud.sensor_origin_[2] << "," << inp_cloud.sensor_origin_[3] << std::endl;
            transG.print();

            pass_.setInputCloud(inp_cloud_ptr);
            pass_.filter(*cloud_filtered);

            if (pcl_count == 1)
            {
                appendCamIndexAndDistance(cloud_filtered,final_cloud,0,transG.getOrigin());
                transformsG.push_back(transG);
                pcl::PointCloud<PointT>::Ptr aCloud(new pcl::PointCloud<PointT > ());
                *aCloud=*cloud_filtered;
                pointClouds.push_back(aCloud);
                pcl::KdTreeFLANN<PointT>::Ptr nnFinder(new pcl::KdTreeFLANN<PointT>);
                nnFinder->setInputCloud(aCloud);
                searchTrees.push_back(nnFinder);

            }
            else if (pcl_count % 5 == 1)
            {
                PointT cpoint;

                std::cerr<<" processing "<<pcl_count<<std::endl;
                for (int p = 0; p < cloud_filtered->size(); p++)
                {
                    bool occluded=false;
                    cpoint = cloud_filtered->points[p];
                    int c;
                        VectorG vpoint(cpoint.x,cpoint.y,cpoint.z);
                    if(!transG.isPointVisible(vpoint)) // if this point is not around the centre of it's own cam, ignore it
                        continue;
                    for (c = 0; c < transformsG.size(); c++)
                    {
                        TransformG ctrans=transformsG[c];
                        if((ctrans.isPointVisible(vpoint))) // is it already visibile in a prev camera? then it might be an outlier
                        {
                            // is it also not occluded in the same camera in which it is visible? only then it will be an outlier
                            pcl::PointCloud<PointT>::Ptr apc=pointClouds[c];
                            pcl::KdTreeFLANN<PointT>::Ptr annFinder=searchTrees[c];
                            int lpt;
                            // if any point in the same frame occludes it, it wont be an outlier
                                VectorG cam2point=vpoint.subtract(ctrans.getOrigin());
                                double distance=cam2point.getNorm();
                                double radiusCyl=0.05;
                                cam2point.normalize();
                                int numPointsInBw=(int)(distance/radiusCyl);
                                set<int> indices;
                                indices.clear();
                            for(lpt=2;lpt<numPointsInBw;lpt++)
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
                                break; // reject this point, must present in prev camera
                            }
                            else
                            {
                                VectorG cam2point=vpoint.subtract(transG.getOrigin());//change to the cam of test point to check if this occluded point is a duplicate
                                double distance=cam2point.getNorm();
                                double radiusCyl=0.05;
                                cam2point.normalize();
                                int numPointsInBw=(int)(distance/radiusCyl);
                                set<int> indices;
                                indices.clear();
                                for (lpt = 2; lpt < numPointsInBw; lpt++)
                                {
                                    VectorG offset = cam2point.multiply(lpt * radiusCyl);
                                    VectorG linePt = offset.add(transG.getOrigin());// change origin to current cam
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
                                occluded = false;
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
                                //visible but occluded by some point in same frame
                                if(occluded)
                                {
                                    std::cerr<<"occlusion repudiation detected "<<numOcclRepudiated++ <<" pcl no:"<< pcl_count<<std::endl;
                                    break;
                                }
                                else
                                    std::cerr<<"occlusion detected "<<numOccluded++ <<" pcl no:"<< pcl_count<<std::endl;

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
                        newPoint.cameraIndex=transformsG.size();
                        newPoint.distance=VectorG(cpoint).subtract(transG.getOrigin()).getNorm();
                        final_cloud->points.push_back(newPoint);
                    }
                    else
                    {
                        rejectCount++;
                    }
                }
                transformsG.push_back(transG);
                pcl::PointCloud<PointT>::Ptr aCloud(new pcl::PointCloud<PointT > ());
                *aCloud=*cloud_filtered;
                pointClouds.push_back(aCloud);
                pcl::KdTreeFLANN<PointT>::Ptr nnFinder(new pcl::KdTreeFLANN<PointT>);
                nnFinder->setInputCloud(aCloud);
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

    bag.close();
    pcl::PCDWriter writer;

    writer.write<pcl::PointXYGRGBCam > ("/home/aa755/VisibilityMerged.pcd", *final_cloud, false);

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


