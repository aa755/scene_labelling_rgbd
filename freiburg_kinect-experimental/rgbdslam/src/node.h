#ifndef RGBD_SLAM_NODE_H_
#define RGBD_SLAM_NODE_H_

#include "ros/ros.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <hogman_minimal/graph/loadEdges3d.h>
 #include "tf/transform_listener.h"



// Searchstructure for descriptormatching
typedef cv::flann::Index cv_flannIndex;


class Node {
  public:
    ///Visual must be CV_8UC1, depth CV_32FC1, id must correspond to the hogman vertex id
    ///detection_mask must be CV_8UC1 with non-zero at potential keypoint locations
    Node(ros::NodeHandle* nh,
         const cv::Mat& visual, const cv::Mat& depth,
         image_geometry::PinholeCameraModel cam_model, 
         cv::Ptr<cv::FeatureDetector> detector,
         cv::Ptr<cv::DescriptorExtractor> extractor,
         cv::Ptr<cv::DescriptorMatcher> matcher, // deprecated!
         const sensor_msgs::PointCloud2ConstPtr& point_cloud,
         unsigned int msg_id=0,
         unsigned int id=0,
         const cv::Mat& detection_mask = cv::Mat());
    //default constructor. TODO: still needed?
    Node(){}

    ///Compute the relative transformation between the nodes
    ///Do either max_ransac_iterations or half of it, 
    ///Iterations with more than half of the initial_matches inlying, count twice
    ///Iterations with more than 80% of the initial_matches inlying, count threefold
    bool getRelativeTransformationTo(const Node& target_node, 
                                     std::vector<cv::DMatch>* initial_matches,
                                     Eigen::Matrix4f& resulting_transformation, 
                                     float& rmse,
                                     std::vector<cv::DMatch>& matches,//for visualization?
                                     unsigned int min_inlier_threshold = 15,
                                     unsigned int max_ransac_iterations = 10000) const;

    // initial_transformation: optional transformation applied to this->pc before
    // using icp
    bool getRelativeTransformationTo_ICP(const Node& target_node,Eigen::Matrix4f& transformation,
            const Eigen::Matrix4f* initial_transformation = NULL);

    bool getRelativeTransformationTo_ICP2(Node& target_node,Eigen::Matrix4f& transformation,
            const Eigen::Matrix4f* initial_transformation = NULL);

    //Send own pointcloud on given topic with given timestamp
    void publish(const char* topic, ros::Time timestamp);
    //void publish2(const char* topic, ros::Time timestamp, Transformation3 transf);
    void publish2(const char* topic, ros::Time timestamp, tf::Transform transform);
    void buildFlannIndex();
    int findPairsFlann(Node& other, vector<cv::DMatch>* matches);


    pcl::PointCloud<pcl::PointXYZRGB> pc_col;
    cv::Mat feature_descriptors_;         ///<descriptor definitions
    std::vector<Eigen::Vector4f> feature_locations_3d_;  ///<backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
    std::vector<cv::KeyPoint> feature_locations_2d_; ///<Where in the image are the descriptors
    ros::NodeHandle* nh_;
    unsigned int msg_id_;
    unsigned int id_; ///must correspond to the hogman vertex id

  protected:
    ros::Publisher cloud_pub;
    ros::Publisher cloud_pub2;
    sensor_msgs::PointCloud2 cloudMessage_; //this will be send out in batch mode
    cv_flannIndex* flannIndex;
    image_geometry::PinholeCameraModel cam_model_;  
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    /** remove invalid keypoints (NaN or outside the image) and return the backprojection of valid ones*/
    void projectTo3D(const cv::Mat& depth,
                     std::vector<cv::KeyPoint>& feature_locations_2d,
                     std::vector<Eigen::Vector4f>& feature_locations_3d,
                     const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);


    // helper for ransac
    void computeInliersAndError(const std::vector<cv::DMatch>& initial_matches,
                                const Eigen::Matrix4f& transformation,
                                const std::vector<Eigen::Vector4f>& origins,  
                                const std::vector<Eigen::Vector4f>& targets,
                                std::vector<cv::DMatch>& new_inliers, //output var
                                double& mean_error, vector<double>& errors,
                                double squaredMaxInlierDistInM = 0.0009) const; //output var;

};
#endif
