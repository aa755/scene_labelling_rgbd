/* registration main:
 * Create 
 * - a Qt Application 
 * - a ROS Node, 
 * - a ROS listener, listening to and processing kinect data
 */

#include <stdint.h>
#include "pcl/ros/register_point_struct.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include "pcl/io/pcd_io.h"
#include <string>
#include <pcl_ros/io/bag_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pcl/ModelCoefficients.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
    typedef pcl::PointXYZRGB PointT ;
	typedef  pcl::KdTree<PointT> KdTree;
	typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;

int main(int argc, char** argv)
{
//  ros::init(argc, argv,"hi");
float tolerance = 0.005;
//    rosbag::Bag bag;
 //   std::cerr<<"opening "<<argv[1]<<endl;
//    bag.open(argv[1], rosbag::bagmode::Read);
     pcl::PointCloud<PointT>::Ptr final_cloud (new pcl::PointCloud<PointT> ()),cloud_filtered (new pcl::PointCloud<PointT> ());


    int tf_count =0;
    int pcl_count=0;

pcl_ros::BAGReader reader;
  if (!reader.open (argv[1], "/rgbdslam/my_clouds"))
  {
    ROS_ERROR ("Couldn't read file ");
    return (-1);
  }
  sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
            pcl::PointCloud<PointT> inp_cloud;

 pcl::PassThrough<PointT> pass_;
  do
  {
    cloud_blob_prev = cloud_blob;
    cloud_blob = reader.getNextCloud ();
    ros::Time ptime=cloud_blob->header.stamp;

    if (cloud_blob_prev != cloud_blob)
    {
      pcl::fromROSMsg (*cloud_blob, inp_cloud);
      //ROS_INFO ("PointCloud with %d data points and frame %s (%f) received.", (int)cloud.points.size (), cloud.header.frame_id.c_str (), cloud.header.stamp.toSec ());
      pcl_count++;
    }

//    rosbag::View view(bag, rosbag::TopicQuery("/rgbdslam/my_clouds"));
    

/*        rosbag::View view_tf(bag, rosbag::TopicQuery("/tf"),ptime-ros::Duration(0,1),ptime+ros::Duration(0,100000000));
        //std::cerr<<(view_tf.size())<<endl;
        std::cerr<<ptime<<endl;;
//        std::cerr<<"qid:"<<pcl_ptr->header.seq<<endl;;
        tf_count=0;

        tf::Transform final_tft;


        BOOST_FOREACH(rosbag::MessageInstance const mtf, view_tf)
        {
            tf::tfMessageConstPtr tf_ptr = mtf.instantiate<tf::tfMessage>();
            assert(tf_ptr!=NULL);
            std::vector<geometry_msgs::TransformStamped> bt;
            tf_ptr->get_transforms_vec(bt);
            tf::Transform tft(getQuaternion(bt[0].transform.rotation),getVector3(bt[0].transform.translation));
            final_tft=tft;

         //  transG.print();
            if(ptime==bt[0].header.stamp)
            {
                tf_count++;
                std::cerr<<"tf qid:"<<bt[0].header.seq<<endl;
            }
            assert(tf_count<=1);
        }
*/
      //  if (tf_count == 1)
        {
   //         TransformG transG(final_tft);
            pcl::PointCloud<PointT>::Ptr inp_cloud_ptr(new pcl::PointCloud<PointT > (inp_cloud));
           // applyFilters(inp_cloud_ptr,cloud_filtered);

    pass_.setInputCloud (inp_cloud_ptr);
    pass_.filter (*cloud_filtered);
            if(pcl_count==1)
                *final_cloud = *cloud_filtered;
            else if (pcl_count % 5 == 1) {


      /*          pcl::PointCloud<PointT> new_cloud;
                new_cloud.points.resize(cloud_filtered->points.size());
                new_cloud.header = cloud_filtered->header;
                KdTreePtr clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
                initTree(0, clusters_tree_);
                clusters_tree_->setInputCloud(final_cloud);
                std::vector<int> nn_indices;
                std::vector<float> nn_distances;
                int nr_p = 0;
                ROS_INFO("Input cloud size = %d", cloud_filtered->points.size());
                for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {

                    if (!clusters_tree_->radiusSearch(cloud_filtered->points[i], tolerance, nn_indices, nn_distances)) {
                        new_cloud.points[nr_p++] = cloud_filtered->points[i];

                    } else {
                    }

                }
                new_cloud.points.resize(nr_p);
                ROS_INFO("New cloud size = %d", new_cloud.points.size());
                //*combined_cloud_ptr += new_cloud;
*/
                *final_cloud += *cloud_filtered;//new_cloud;
            }
        } /*else {
            std::cerr << "no tf found";
        }*/


    } while (cloud_blob != cloud_blob_prev);

    //applyFilters(final_cloud, cloud_filtered);

//    bag.close();
    pcl::PCDWriter writer;

//    writer.write<PointT > ("/home/aa755/VisibilityMerged.pcd", *cloud_filtered, false);
pcl::io::savePCDFile ("/home/aa755/VisibilityMerged.pcd", *final_cloud, false);

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


