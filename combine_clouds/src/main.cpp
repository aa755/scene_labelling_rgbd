/* registration main:
 * Create 
 * - a Qt Application 
 * - a ROS Node, 
 * - a ROS listener, listening to and processing kinect data
 */
#include "openni_listener.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
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
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_cloud.h"

float sqrG(float y)
{
    return y*y;
}

class VectorG
{
    double v[3];
public:
    VectorG()
    {

    }
    VectorG(double unitX,double unitY, double unitZ , bool normalize_=false)
    {
        v[0]=unitX;
        v[1]=unitY;
        v[2]=unitZ;

        if(normalize_)
        {
            normalize();
        }

    }
    void normalize()
    {
            double norm=getNorm();
            for(int i=0;i<3;i++)
                v[i]=v[i]/norm;
    }

    double getNorm()
    {
        return sqrt(sqrG(v[0])+sqrG(v[1])+sqrG(v[2]));
    }

    double dotProduct(VectorG v2g)
    {
        double sum=0;
        for(int i=0;i<3;i++)
            sum=sum+v[i]*v2g.v[i];
        return sum;
    }

    VectorG subtract(VectorG v2g)
    {
        VectorG out;
        for(int i=0;i<3;i++)
            out.v[i]=v[i]-v2g.v[i];
        return out;

    }

};
boost::numeric::ublas::matrix<double> transformAsMatrix(const tf::Transform& bt)
{
   boost::numeric::ublas::matrix<double> outMat(4,4);

   //  double * mat = outMat.Store();

   double mv[12];
   bt.getBasis().getOpenGLSubMatrix(mv);

   btVector3 origin = bt.getOrigin();

   outMat(0,0)= mv[0];
   outMat(0,1)  = mv[4];
   outMat(0,2)  = mv[8];
   outMat(1,0)  = mv[1];
   outMat(1,1)  = mv[5];
   outMat(1,2)  = mv[9];
   outMat(2,0)  = mv[2];
   outMat(2,1)  = mv[6];
   outMat(2,2) = mv[10];

   outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
   outMat(0,3) = origin.x();
   outMat(1,3) = origin.y();
   outMat(2,3) = origin.z();
   outMat(3,3) = 1;


   return outMat;
}

class TransformG
{
public:
    boost::numeric::ublas::matrix<double>  transformMat;


    TransformG(const tf::Transform& bt)
    {
        transformMat=transformAsMatrix(bt);
    }

    VectorG getXUnitVector()
    {
        return getIthRow(0);
    }

    VectorG getZUnitVector()
    {
        return getIthRow(2);
    }

    VectorG getIthRow(int i)
    {
        return VectorG(transformMat(i,0),transformMat(i,1),transformMat(i,2));
    }

    VectorG getOrigin()
    {
        return VectorG(transformMat(0,3),transformMat(1,3),transformMat(2,3));
    }

    void print()
    {
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
                std::cerr<<transformMat(i,j)<<",";
            std::cerr<<endl;
        }
    }
};

btQuaternion getQuaternion(geometry_msgs::Quaternion q)
{
    return btQuaternion(q.x,q.y,q.z,q.w);
}

btVector3 getVector3(geometry_msgs::Vector3 v)
{
    return btVector3(v.x,v.y,v.z);

}
int main(int argc, char** argv)
{
//  ros::init(argc, argv,"hi");

    rosbag::Bag bag;
    std::cerr<<"opening "<<argv[1]<<endl;
    bag.open(argv[1], rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    topics.push_back(std::string("/rgbdslam/my_clouds"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));


    int count =0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2ConstPtr pcl_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if(pcl_ptr==NULL)
            std::cerr<<"null1"<<endl;
        else
            std::cerr<<count<<"time_pcl"<<pcl_ptr->header.stamp<<", frameid="/*<<pcl_ptr->header.frame_id*/<<endl;


        tf::tfMessageConstPtr tf_ptr = m.instantiate<tf::tfMessage>();
        std::vector<geometry_msgs::TransformStamped> bt;

        if(tf_ptr==NULL)
            std::cerr<<"null2"<<endl;
        else
        {
          //  tf::StampedTransform tf1(bt[0]);
            tf_ptr->get_transforms_vec(bt);
            TransformG transG(tf::Transform(getQuaternion(bt[0].transform.rotation),getVector3(bt[0].transform.translation)));
        std::cerr<<count<<"time_tf"<<bt[0].header.stamp<<", frameid="/*<<pcl_ptr->header.frame_id*/<<endl;
            transG.print();
        }
        count++;
        
    }

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


