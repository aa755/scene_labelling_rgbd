#include "graph_manager.h"
#include <hogman_minimal/stuff/macros.h>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <rgbdslam/CloudTransforms.h>
#include "pcl_ros/transforms.h"


void GraphManager::mat2dist(Eigen::Matrix4f t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}


void GraphManager::mat2RPY(Eigen::Matrix4f t, double& roll, double& pitch, double& yaw)
{
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}

Transformation3 eigen2Hogman(const Eigen::Matrix4f eigen_mat) {
    std::clock_t starttime=std::clock();

    Eigen::Affine3f eigen_transform(eigen_mat);
    Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
    Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
    Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
            eigen_quat.w());
    Transformation3 result(translation, rotation);

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
    return result;
}

tf::Transform hogman2TF(const Transformation3 hogman_trans) {
    std::clock_t starttime=std::clock();

    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(hogman_trans.translation().x());
    translation.setY(hogman_trans.translation().y());
    translation.setZ(hogman_trans.translation().z());

    tf::Quaternion rotation;
    rotation.setX(hogman_trans.rotation().x());
    rotation.setY(hogman_trans.rotation().y());
    rotation.setZ(hogman_trans.rotation().z());
    rotation.setW(hogman_trans.rotation().w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    return result;

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 

}

void printTransform(const char* name, const tf::Transform t) {
    ROS_DEBUG_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_DEBUG_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}


GraphManager::GraphManager(ros::NodeHandle nh) :
    freshlyOptimized_(false),
    nh_(nh), time_of_last_transform_(ros::Time()),
    optimizer_(0), reset_request_(false),
    last_batch_update_(std::clock()),
    marker_id(0),
    batch_processing_runs_(false)
{
    std::clock_t starttime=std::clock();

    int numLevels = 3;
    int nodeDistance = 2;
    optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("camera_pose_graph", 1);
    kinect_transform_.setRotation(tf::Quaternion::getIdentity());
    timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastTransform, this);
    transform_pub_ = nh.advertise<rgbdslam::CloudTransforms>("/rgbdslam/transforms", 1);

    ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

GraphManager::~GraphManager() {
    delete (optimizer_);
}

void GraphManager::drawFeatureFlow(cv::Mat& canvas, 
                                   cv::Scalar line_color,
                                   cv::Scalar circle_color){
    std::clock_t starttime=std::clock();
    ROS_DEBUG("Number of features to draw: %d", (int)matches_.size());

    const double pi_fourth = 3.14159265358979323846 / 4.0;
    const int line_thickness = 1;
    const int circle_radius = 6;
    if(graph_.size() < 2) return; //feature flow is only available between at least two nodes

    Node& earliernode = graph_[0];//graph_.size()-2; //compare current to previous
    Node& newernode = graph_[graph_.size()-1];

    for(unsigned int mtch = 0; mtch < matches_.size(); mtch++) {
        cv::Point2f p,q; //TODO: Use sub-pixel-accuracy
        unsigned int newer_idx = matches_[mtch].queryIdx;
        unsigned int earlier_idx = matches_[mtch].trainIdx;
        q = newernode.feature_locations_2d_[newer_idx].pt;
        p = earliernode.feature_locations_2d_[earlier_idx].pt;

        double angle;    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse;    hypotenuse = cv::norm(p-q);
        if(hypotenuse < 1.5){ //encircle near-stationary points
            cv::line(canvas, p, q, line_color, line_thickness, 8);
            cv::circle(canvas, p, circle_radius, circle_color, line_thickness, 8);
        }else { /* Connect the points */
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
            /* Now draw the tips of the arrow.  */
            p.x =  (q.x + 4 * cos(angle + pi_fourth));
            p.y =  (q.y + 4 * sin(angle + pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
            p.x =  (q.x + 4 * cos(angle - pi_fourth));
            p.y =  (q.y + 4 * sin(angle - pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
        }
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}


/// max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently
std::vector<int> GraphManager::getPotentialEdgeTargets(const Node& new_node, int max_targets){
    std::vector<int> ids_to_link_to;
    if(graph_.size() == 0){
        ROS_WARN("Do not call this function as long as the graph is empty");
        return ids_to_link_to;
    }
    if(max_targets < 0||max_targets > (int)graph_.size()){
        max_targets = graph_.size(); //can't have more edges than nodes
    }else if(max_targets == 0){
        ids_to_link_to.push_back(0);
        return ids_to_link_to; //only compare to first frame
    } else if(max_targets == 1){
        ids_to_link_to.push_back(graph_.size()-1); 
        return ids_to_link_to; //only compare to previous frame
    } 

    double max_id_plus1 = (double)graph_.size();
    double id = 0; //always start with the first
    while((id+0.5) < max_id_plus1){
        ids_to_link_to.push_back((int)(id+0.5)); //implicit rounding
        id += (max_id_plus1/(double)max_targets);
    }
    return ids_to_link_to;
}

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node new_node) {


    std::clock_t starttime=std::clock();
    if(reset_request_){
        int numLevels = 3;
        int nodeDistance = 2;
        marker_id =0;
        time_of_last_transform_= ros::Time();
        last_batch_update_=std::clock();
        delete optimizer_;
        optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
        graph_.clear();
        matches_.clear();
        freshlyOptimized_= false;
        reset_request_ = false;
    }


    if (new_node.feature_locations_2d_.size() <= 5){
        ROS_DEBUG("found only %i features on image, node is not included",(int)new_node.feature_locations_2d_.size());
        return false;
    }

    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node.id_ = graph_.size();

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        new_node.buildFlannIndex(); // create index so that next nodes can use it

//        graph_.insert(make_pair(new_node.id_, new_node)); //store
        graph_[new_node.id_] = new_node;
        optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0)); //fix at origin
        return true;
    }


    unsigned int num_edges_before = optimizer_->edges().size(); 


    std::vector<cv::DMatch> initial_matches;
    ROS_DEBUG("Graphsize: %d", (int) graph_.size());
    marker_id = 0; //overdraw old markers


    Eigen::Matrix4f ransac_trafo, final_trafo;

    bool edge_added_to_base;
    std::vector<int> vertices_to_comp = getPotentialEdgeTargets(new_node, -1); //vernetzungsgrad
    int id_of_id = vertices_to_comp.size() -1;
    for (; id_of_id >=0; id_of_id--){ //go from the back, so the last comparison is with the first node. The last comparison is visualized.
     initial_matches = processNodePair(new_node, graph_[vertices_to_comp[id_of_id]],edge_added_to_base,ransac_trafo, final_trafo);
        //initial_matches = processNodePair(new_node, graph_rit->second);
        //What if the node has not been added? visualizeFeatureFlow3D(graph_rit->second, new_node, initial_matches, matches_, marker_id++);
    }
    id_of_id++;//set back to last valid id

    if (optimizer_->edges().size() > num_edges_before) {

        new_node.buildFlannIndex();
        graph_[new_node.id_] = new_node;

        ROS_DEBUG("Added Node, new Graphsize: %i", (int) graph_.size());

        optimizeGraph();
        //make the transform of the last node known
        ros::TimerEvent unused;
        broadcastTransform(unused);
        visualizeGraphEdges();
        visualizeGraphNodes();
        visualizeFeatureFlow3D(graph_[vertices_to_comp[id_of_id]], new_node, initial_matches, matches_, marker_id++);
    }else{
        ROS_WARN("##### could not find link for PointCloud!");
        matches_.clear();
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
    return (optimizer_->edges().size() > num_edges_before);
}

std::vector<cv::DMatch> GraphManager::processNodePair(Node& new_node, Node& old_node, bool& edge_added,Eigen::Matrix4f& ransac_transform, Eigen::Matrix4f& final_trafo){

    std::clock_t starttime=std::clock();
    const unsigned int hit_threshold = 50; // minimal number of hits to be a valid candidate for a link
    std::vector<cv::DMatch> initial_matches;
    Eigen::Matrix4f icp_trafo;
    Eigen::Matrix4f icp_trafo2;

    AIS::LoadedEdge3D edge;
    float rmse;
    new_node.findPairsFlann(old_node, &initial_matches); 
    ROS_DEBUG("found %i inital matches",(int) initial_matches.size());
    if (initial_matches.size() >= hit_threshold){
        bool valid_trafo = new_node.getRelativeTransformationTo(old_node,&initial_matches, ransac_transform, rmse, matches_);
        if (!valid_trafo){
            ROS_INFO("GraphManager: Found no valid trafo, but had initially %d hits",(int) initial_matches.size());
            edge_added = false;
        }
        else  {


            // improve transformation by using the generalized ICP
           // std::clock_t starttime_gicp = std::clock();
          //  new_node.getRelativeTransformationTo_ICP(old_node,icp_trafo, &ransac_transform);
          //  cout << "complete time for ICP: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC) << endl;


         //   cout << "old icp " << icp_trafo << endl;
         //   starttime_gicp = std::clock();
            //new_node.getRelativeTransformationTo_ICP2(old_node,icp_trafo2, &ransac_transform);
         //   cout << "complete time for ICP2: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC) << endl;


            //cout << "new icp " << icp_trafo2 << endl;


          //  ROS_INFO_STREAM("zzz  icp trafo: " << endl << icp_trafo << endl);
          //  ROS_INFO_STREAM("zzz  icp trafo2: " << endl << icp_trafo2 << endl);


            final_trafo = ransac_transform ; //*icp_trafo2;


            edge.id1 = old_node.id_;//and we have a valid transformation
            edge.id2 = new_node.id_; //since there are enough matching features,
            edge.mean = eigen2Hogman(final_trafo);//we insert an edge between the frames
            edge.informationMatrix = Matrix6::eye(1.0); //TODO: What do we do about the information matrix? Scale with rmse? inlier_count)

            edge_added = addEdgeToHogman(edge, isBigTrafo(final_trafo));

            if (edge_added){
                ROS_DEBUG("Adding Edge between %i and %i. Inlier: %i, error: %f",edge.id1,edge.id2,(int) matches_.size(),rmse);
            }
        }
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
    return initial_matches;
}

// true iff edge qualifies for generating a new vertex
bool GraphManager::isBigTrafo(const Eigen::Matrix4f& t){

    double roll, pitch, yaw, dist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;


    double max_angle = max(roll,max(pitch,yaw));

    // at least 10cm or 5deg
    return (dist > 0.1 || max_angle > 5);

}


void GraphManager::visualizeFeatureFlow3D(const Node& earlier_node,
                                          const Node& newer_node,
                                          const std::vector<cv::DMatch>& all_matches,
                                          const std::vector<cv::DMatch>& inlier_matches,
                                          unsigned int marker_id,
                                          bool draw_outlier) const{
    std::clock_t starttime=std::clock();
    if (ransac_marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking

        visualization_msgs::Marker marker_lines;

        marker_lines.header.frame_id = "/openni_rgb_optical_frame";
        marker_lines.ns = "ransac_markers";
        marker_lines.header.stamp = ros::Time::now();
        marker_lines.action = visualization_msgs::Marker::ADD;
        marker_lines.pose.orientation.w = 1.0;
        marker_lines.id = marker_id;
        marker_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_lines.scale.x = 0.002;
        
        std_msgs::ColorRGBA color_red  ;  //red outlier
        color_red.r = 1.0;
        color_red.a = 1.0;
        std_msgs::ColorRGBA color_green;  //green inlier, newer endpoint
        color_green.g = 1.0;
        color_green.a = 1.0;
        std_msgs::ColorRGBA color_yellow;  //yellow inlier, earlier endpoint
        color_yellow.r = 1.0;
        color_yellow.g = 1.0;
        color_yellow.a = 1.0;
        std_msgs::ColorRGBA color_blue  ;  //red-blue outlier
        color_blue.b = 1.0;
        color_blue.a = 1.0;

        marker_lines.color = color_green; //just to set the alpha channel to non-zero

        AISNavigation::PoseGraph3D::Vertex* earlier_v; //used to get the transform
        AISNavigation::PoseGraph3D::Vertex* newer_v; //used to get the transform
        AISNavigation::PoseGraph3D::VertexIDMap v_idmap = optimizer_->vertices();
        // end of initialization
        ROS_DEBUG("Matches Visualization start");

        // write all inital matches to the line_list
        marker_lines.points.clear();//necessary?

        if (draw_outlier)
        {
            for (unsigned int i=0;i<all_matches.size(); i++){
                int newer_id = all_matches.at(i).queryIdx; //feature id in newer node
                int earlier_id = all_matches.at(i).trainIdx; //feature id in earlier node

                earlier_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[earlier_node.id_]);
                newer_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[newer_node.id_]);

                //Outliers are red (newer) to blue (older)
                marker_lines.colors.push_back(color_red);
                marker_lines.colors.push_back(color_blue);

                marker_lines.points.push_back(
                        pointInWorldFrame(newer_node.feature_locations_3d_[newer_id],
                                newer_v->transformation));
                marker_lines.points.push_back(
                        pointInWorldFrame(earlier_node.feature_locations_3d_[earlier_id],
                                earlier_v->transformation));
            }
        }


        for (unsigned int i=0;i<inlier_matches.size(); i++){
            int newer_id = inlier_matches.at(i).queryIdx; //feature id in newer node
            int earlier_id = inlier_matches.at(i).trainIdx; //feature id in earlier node

            earlier_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[earlier_node.id_]);
            newer_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[newer_node.id_]);


            //inliers are green (newer) to blue (older)
            marker_lines.colors.push_back(color_green);
            marker_lines.colors.push_back(color_blue);

            marker_lines.points.push_back(
                    pointInWorldFrame(newer_node.feature_locations_3d_[newer_id],
                            newer_v->transformation));
            marker_lines.points.push_back(
                    pointInWorldFrame(earlier_node.feature_locations_3d_[earlier_id],
                            earlier_v->transformation));
        }



        ransac_marker_pub_.publish(marker_lines);
        ROS_DEBUG_STREAM("Published  " << marker_lines.points.size()/2 << " lines");
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

//do spurious type conversions
geometry_msgs::Point pointInWorldFrame(Eigen::Vector4f point3d, 
        Transformation3 transf){
    Vector3f tmp(point3d[0], point3d[1], point3d[2]);
    tmp = transf * tmp; //transform to world frame
    geometry_msgs::Point p;
    p.x = tmp.x(); 
    p.y = tmp.y(); 
    p.z = tmp.z();
    return p;
}


void GraphManager::visualizeGraphEdges() const {
    std::clock_t starttime=std::clock();

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one



        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 0.5f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        AISNavigation::PoseGraph3D::Vertex* v; //used in loop
        AISNavigation::PoseGraph3D::EdgeSet::iterator edge_iter = optimizer_->edges().begin();
        int counter = 0;
        for(;edge_iter != optimizer_->edges().end(); edge_iter++, counter++) {
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->from());
            point.x = v->transformation.translation().x();
            point.y = v->transformation.translation().y();
            point.z = v->transformation.translation().z();
            edges_marker.points.push_back(point);
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->to());
            point.x = v->transformation.translation().x();
            point.y = v->transformation.translation().y();
            point.z = v->transformation.translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::visualizeGraphNodes() const {
    std::clock_t starttime=std::clock();

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.002;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Vector3f origin(0.0,0.0,0.0);
        Vector3f x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Vector3f y_axis(0.0,0.2,0.0);
        Vector3f z_axis(0.0,0.0,0.2);
        Vector3f tmp; //the transformed endpoints
        int counter = 0;
        AISNavigation::PoseGraph3D::Vertex* v; //used in loop
        AISNavigation::PoseGraph3D::VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
        for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*vertex_iter).second);
            //v->transformation.rotation().x()+ v->transformation.rotation().y()+ v->transformation.rotation().z()+ v->transformation.rotation().w();
            tmp = v->transformation * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->transformation * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->transformation * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->transformation * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

bool GraphManager::addEdgeToHogman(AIS::LoadedEdge3D edge, bool largeEdge) {
    std::clock_t starttime=std::clock();
    freshlyOptimized_ = false;


    AIS::PoseGraph3D::Vertex* v1 = optimizer_->vertex(edge.id1);
    AIS::PoseGraph3D::Vertex* v2 = optimizer_->vertex(edge.id2);


    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("edge to new vertex is to short, vertex will not be inserted");
            //return false;
        }
    }

    if (!v1) {
        v1 = optimizer_->addVertex(edge.id1, Transformation3(), Matrix6::eye(1.0));
        assert(v1);
    }
    if (!v2) {
        v2 = optimizer_->addVertex(edge.id2, Transformation3(), Matrix6::eye(1.0));
        assert(v2);
    }
    optimizer_->addEdge(v1, v2, edge.mean, edge.informationMatrix);
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 

    return true;

}

void GraphManager::optimizeGraph(){
    std::clock_t starttime=std::clock();
    const int iterations = 10;
    int currentIt = optimizer_->optimize(iterations, true);

  double chi2 = optimizer_->chi2();
  ROS_INFO_STREAM("nodes= " << optimizer_->vertices().size() << "\t edges= "
                  << optimizer_->edges().size() << "\t chi2= " << chi2
                  << "\t iterations= " << currentIt);

  freshlyOptimized_ = true;

  AISNavigation::PoseGraph3D::Vertex* v = optimizer_->vertex(optimizer_->vertices().size()-1);
  kinect_transform_ =  hogman2TF(v->transformation);

  //publish the corrected transforms to the visualization module every five seconds
  if( ((std::clock()-last_batch_update_) / (double)CLOCKS_PER_SEC) > 2){
      publishCorrectedTransforms();
      last_batch_update_ = std::clock();
  }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::broadcastTransform(const ros::TimerEvent& ){

    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));


    world2cam_ = cam2rgb*kinect_transform_;
    //printTransform("kinect", kinect_transform_);
    time_of_last_transform_ = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(world2cam_, time_of_last_transform_,
                      "/openni_camera", "/slam_transform"));

    if(!batch_processing_runs_)
        br_.sendTransform(tf::StampedTransform(cam2rgb, time_of_last_transform_,
                          "/openni_camera", "/batch_transform"));

    //visualize the transformation
    std::stringstream ss;
    Eigen::Matrix4f transform;
    pcl_ros::transformAsMatrix(kinect_transform_, transform);
    ss << "<b>Current Camera Transformation w.r.t. the Initial Frame</b>";
    ss << "<pre>" <<  transform << "</pre>";
    QString mystring(ss.str().c_str());
    emit newTransformationMatrix(mystring);

}

/**
 * Publish the updated transforms for the graph node resp. clouds
 */
void GraphManager::publishCorrectedTransforms(){
    std::clock_t starttime=std::clock();
    //fill message
    rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        tf::Transform trans = hogman2TF(v->transformation);
        geometry_msgs::Transform trans_msg;
        tf::transformTFToMsg(trans,trans_msg);
        msg.transforms.push_back(trans_msg);
        msg.ids.push_back(graph_[i].msg_id_);
    }
    msg.header.stamp = ros::Time::now();

    if (transform_pub_.getNumSubscribers() > 0)
        transform_pub_.publish(msg);
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::reset(){
    reset_request_ = true;
}

void GraphManager::sendAllClouds(){
    std::clock_t starttime=std::clock();
    ROS_WARN("Sending out all clouds");
    batch_processing_runs_ = true;
    tf::Transform  world2cam;
    //fill message
    rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        if(!v){ 
            ROS_FATAL("Nullpointer im Graph an Position %i!", i);
            continue;
        }
        tf::Transform transform = hogman2TF(v->transformation);

        tf::Transform cam2rgb;
        cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
        cam2rgb.setOrigin(tf::Point(0,-0.04,0));

        world2cam = cam2rgb*transform;
        ros::Time time_of_transform = ros::Time::now();
        ROS_WARN("Sending out transform %i", i);
        br_.sendTransform(tf::StampedTransform(world2cam, time_of_transform,
                          "/openni_camera", "/batch_transform"));
        ROS_WARN("Sending out cloud %i", i);
        graph_[i].publish("/batch_transform", time_of_transform);


		graph_[i].publish2("/my_transform", time_of_transform,world2cam);
    }

    batch_processing_runs_ = false;
    emit sendFinished();
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}
