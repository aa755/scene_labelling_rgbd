/*
 * graph_manager.h
 *
 *  Created on: 19.01.2011
 *      Author: hess
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "node.h"
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <hogman_minimal/graph/loadEdges3d.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <QObject>
#include <QString>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
//#define ROSCONSOLE_SEVERITY_INFO

namespace AIS = AISNavigation;

class GraphManager : public QObject {
    Q_OBJECT
    signals:
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    void sendFinished();
    public:
    GraphManager(ros::NodeHandle);
    ~GraphManager();

    std::vector<cv::DMatch> processNodePair(Node& new_node, Node& old_node, bool& edge_added,Eigen::Matrix4f& ransac_transform, Eigen::Matrix4f& final_trafo);

    /// Add new node to the graph.
    /// Node will be included, if a valid transformation to one of the former nodes
    /// can be found. If appropriate, the graph is optimized
    bool addNode(Node newNode); 
    ///Draw the features's motions onto the canvas
    ///for the edge computed in the last call of hogmanEdge.
    ///This should always be edge between the first and the last inserted nodes
    void drawFeatureFlow(cv::Mat& canvas, 
                         cv::Scalar line_color = cv::Scalar(255,0,0,0), 
                         cv::Scalar circle_color = cv::Scalar(0,0,255,0)); 
    bool freshlyOptimized_;
    ros::NodeHandle nh_; // public to give it to the nodes
    ros::Time time_of_last_transform_;
    tf::Transform  world2cam_;
    std::map<int, Node> graph_;

    public slots:
    void reset();
    ///iterate over all Nodes, sending their transform and pointcloud
    void sendAllClouds(); 
    


protected:


    std::vector < cv::DMatch > matches_;
    /// The parameter max_targets determines how many potential edges are wanted
    /// max_targets < 0: No limit
    /// max_targets = 0: Compare to first frame only
    /// max_targets = 1: Compare to previous frame only
    /// max_targets > 1: Select intelligently
    std::vector<int> getPotentialEdgeTargets(const Node& new_node, int max_targets);
    void optimizeGraph();
    void initializeHogman();
    bool addEdgeToHogman(AIS::LoadedEdge3D edge, bool good_edge);


    ///Send markers to visualize the graph edges (cam transforms) in rviz (if somebody subscribed)
    void visualizeGraphEdges() const;
    ///Send markers to visualize the graph nodes (cam positions) in rviz (if somebody subscribed)
    void visualizeGraphNodes() const;
    ///Send markers to visualize the last matched features in rviz (if somebody subscribed)
    void visualizeFeatureFlow3D(const Node& earlier_node,
                                const Node& later_node,
                                const std::vector<cv::DMatch>& all_matches, 
                                const std::vector<cv::DMatch>& inlier_matches,
                                unsigned int marker_id = 0,
                                bool draw_outlier = true) const;
    ///Send the transform between openni_camera (the initial position of the cam)
    ///and the cumulative motion. 
    ///This is called periodically by a ros timer and after each optimizer run
    void broadcastTransform(const ros::TimerEvent& event);

    ///Batch send all transforms in graph
    void publishCorrectedTransforms();


    AIS::GraphOptimizer3D* optimizer_;

    ros::Publisher marker_pub_; 
    ros::Publisher transform_pub_;
    ros::Publisher ransac_marker_pub_;

    ros::Timer timer_;
    tf::TransformBroadcaster br_;
    tf::Transform kinect_transform_;


    // true if translation > 10cm or largest euler-angle>5 deg
    // used to decide if the camera has moved far enough to generate a new nodes
    bool isBigTrafo(const Eigen::Matrix4f& t);

    /// get euler angles from 4x4 homogenous
    void mat2RPY(Eigen::Matrix4f t, double& roll, double& pitch, double& yaw);
    /// get translation-distance from 4x4 homogenous
    void mat2dist(Eigen::Matrix4f t, double &dist);


    bool reset_request_;
    std::clock_t last_batch_update_;
    unsigned int marker_id;
    bool batch_processing_runs_;

};

geometry_msgs::Point pointInWorldFrame(Eigen::Vector4f point3d, Transformation3 transf);
#endif /* GRAPH_MANAGER_H_ */
