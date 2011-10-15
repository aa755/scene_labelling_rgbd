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
typedef pcl::PointXYZRGBCamSL PointT;
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

    std::vector<TransformG> transformsG;
    std::vector<pcl::PointCloud<PointT>::Ptr> pointClouds;
    std::vector<pcl::KdTreeFLANN<PointT>::Ptr> searchTrees;
    pcl::PCDWriter writer;
  std::vector<int> k_indices;
  std::vector<float> k_distances;

    rosbag::Bag reader;
    rosbag::View view;
    rosbag::View::iterator it;
    bool check;

  try
    {
      reader.open (argv[1], rosbag::bagmode::Read);
      view.addQuery (reader, rosbag::TopicQuery ("/rgbdslam/my_clouds"));

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
    pcl::PointCloud<pcl::PointXYZRGBCamSL> inp_cloud;

    int rejectCount=0;
    pcl::PassThrough<pcl::PointXYZRGBCamSL> pass_;
        cloud_blob_prev = cloud_blob;
	if (it != view.end ())
	  {
	    cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
	    ++it;
	  }
       cloud_blob = cloud_blob_temp; 
        ros::Time ptime = cloud_blob->header.stamp;

            pcl::fromROSMsg(*cloud_blob, inp_cloud);
            //ROS_INFO ("PointCloud with %d data points and frame %s (%f) received.", (int)cloud.points.size (), cloud.header.frame_id.c_str (), cloud.header.stamp.toSec ());
            pcl_count++;
 
    TransformG transForm = readTranform(argv[1]);

    for(int i=0;i<3;i++)
        inp_cloud.sensor_origin_(i)=transForm.transformMat(i,3);
    
            writer.write<pcl::PointXYZRGBCamSL > (std::string(argv[1])+std::string(".pcd"), inp_cloud, true);

    bag.close();

}


