#include "includes/CombineUtils.h"
#include <iostream>
#include<set>
using namespace std;

//template class pcl::ExtractIndices<pcl::PointXYGRGBCam>;

void filterBasedOnCam(pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr in,pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr out, vector<TransformG> & trasforms)
{
   double radius=0.1;
   pcl::PointXYGRGBCam cpoint;
    out->header=in->header;
    set<int> indices;
  pcl::KdTreeFLANN<pcl::PointXYGRGBCam> nnFinder;
  nnFinder.setInputCloud(in);
  std::vector<int> k_indices;
  std::vector<float> k_distances;
  int numNeighbors;
  int minDist;
  int minCamIndex;
  float distance;
        pcl::ExtractIndices<pcl::PointXYGRGBCam> extract;
        pcl::PointIndices::Ptr outliers (new pcl::PointIndices ());
    for(int i=0;i<in->size();i++)
    {
        std::cerr<<i<<" of "<<in->size();

        cpoint = in->points[i];
        VectorG vpoint(cpoint.x,cpoint.y,cpoint.z);
        minDist=100000;
        minCamIndex=-1;
        for(int t=0;t<trasforms.size();t++)
        {
            // among shots from which it is visible, find the shot in which it is nearest to the camera
            if(!trasforms[t].isPointVisible(vpoint))
                continue;
            distance=trasforms[t].getDistanceFromOrigin(vpoint);
            if(distance<minDist)
            {
                minDist=distance;
                minCamIndex=t;
            }
        }
        if(minCamIndex==-1)
        {
            std::cerr<<" point not visible in any cam"<< endl;
            continue;
        }
        numNeighbors=nnFinder.radiusSearch(i,radius,k_indices,k_distances,20);
        std::cerr<<" cam: "<< minCamIndex<<","<<numNeighbors<<" neighbors"<<endl;
        //remove neighbors from all other cams
        
        for(int nn=0;nn<numNeighbors;nn++)
        {

            if(in->points[k_indices[nn]].cameraIndex!=minCamIndex)
                indices.insert(k_indices[nn]);
        }
        k_indices.clear();
        k_distances.clear();


    }
        set<int>::iterator iter;
        for(iter=indices.begin();iter!=indices.end();iter++)
            outliers->indices.push_back(*iter);
        
    extract.setInputCloud (in);
    extract.setIndices (outliers);
    extract.setNegative (false);
    extract.filter (*out);

}

int
main(int argc, char** argv)
{
    //  ros::init(argc, argv,"hi");

    rosbag::Bag bag;
    std::cerr << "opening " << argv[1] << std::endl;
    bag.open(argv[1], rosbag::bagmode::Read);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());// cloud_transformed(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYGRGBCam > ()),cloud_wCamIndices(new pcl::PointCloud<pcl::PointXYGRGBCam > ()),final_wCamIndices(new pcl::PointCloud<pcl::PointXYGRGBCam > ());

    int tf_count = 0;
    int pcl_count = 0;

    std::vector<TransformG> transformsG;
    pcl_ros::BAGReader reader;
    if (!reader.open(argv[1], "/rgbdslam/my_clouds"))
    {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
    pcl::PointCloud<PointT> inp_cloud;

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
                appendCamIndex(cloud_filtered,cloud_wCamIndices,0);
                *final_cloud = *cloud_wCamIndices;
                transformsG.push_back(transG);
            }
            else if (pcl_count % 5 == 1)
            {
                appendCamIndex(cloud_filtered,cloud_wCamIndices,transformsG.size());
                *final_cloud += *cloud_wCamIndices;
                transformsG.push_back(transG);
            }
        }
        else
        {
            std::cerr << "no tf found";
        }



    }
    while (cloud_blob != cloud_blob_prev);

//    applyFilters(final_cloud, cloud_filtered);
    filterBasedOnCam(final_cloud,final_wCamIndices,transformsG);


    bag.close();
    pcl::PCDWriter writer;

    writer.write<pcl::PointXYGRGBCam > ("/home/aa755/VisibilityMerged.pcd", *final_wCamIndices, false);


}


