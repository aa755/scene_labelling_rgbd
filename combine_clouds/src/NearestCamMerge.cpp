#include "CombineUtils.h"


void appendCamIndex(pcl::PointCloud<PointT>::Ptr in,pcl::PointCloud<scene_processing::PointXYGRGBCam>::Ptr out,int camIndex)
{
    out->header=in->header;
    out->points.resize(in->size());
    for(int i=0;i<in->size();i++)
    {
        out->points[i].x=in->points[i].x;
        out->points[i].y=in->points[i].y;
        out->points[i].z=in->points[i].z;
        out->points[i].rgb=in->points[i].rgb;
        out->points[i].cameraIndex=camIndex;
    }
}

int
main(int argc, char** argv)
{
    //  ros::init(argc, argv,"hi");

    rosbag::Bag bag;
    std::cerr << "opening " << argv[1] << std::endl;
    bag.open(argv[1], rosbag::bagmode::Read);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());// cloud_transformed(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<scene_processing::PointXYGRGBCam>::Ptr final_cloud(new pcl::PointCloud<scene_processing::PointXYGRGBCam > ()),cloud_wCamIndices(new pcl::PointCloud<scene_processing::PointXYGRGBCam > ());

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

    bag.close();
    pcl::PCDWriter writer;

 //   writer.write<PointT > ("/home/aa755/VisibilityMerged.pcd", *cloud_filtered, false);


}


