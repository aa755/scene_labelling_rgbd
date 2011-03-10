#include "node.h"
#include <cmath>
#include <ctime>
#include <Eigen/Geometry>
#include <pcl/common/transformation_from_correspondences.h>
#include "pcl/ros/conversions.h"
#include "pcl/point_types.h"
#include <math.h>
#include <fstream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>



static boost::numeric::ublas::matrix<double> transformAsMatrix(const tf::Transform& bt) 
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
};

Node::Node(ros::NodeHandle* nh,
        const cv::Mat& visual, const cv::Mat& depth,
        image_geometry::PinholeCameraModel cam_model,
        cv::Ptr<cv::FeatureDetector> detector,
        cv::Ptr<cv::DescriptorExtractor> extractor,
        cv::Ptr<cv::DescriptorMatcher> matcher,
        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
        unsigned int msg_id,
        unsigned int id,
        const cv::Mat& detection_mask):
        nh_(nh),
        msg_id_(msg_id),
        id_(id),
        cloudMessage_(*point_cloud),
        cam_model_(cam_model),
        matcher_(matcher)
{
    std::clock_t starttime=std::clock();

    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "Feature detection runtime: " << ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC );
    ROS_INFO("Found %d Keypoints", (int)feature_locations_2d_.size());

    cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>("/rgbdslam/batch_clouds",20);
    cloud_pub2 = nh_->advertise<sensor_msgs::PointCloud2>("/rgbdslam/my_clouds",20);

    // get pcl::Pointcloud to extract depthValues a pixel positions
    std::clock_t starttime5=std::clock();
    // TODO: This takes 0.1 seconds and is not strictly necessary
    //pcl::fromROSMsg(*point_cloud,pc);
    pcl::fromROSMsg(*point_cloud,pc_col);
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime5) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "projection runtime: " << ( std::clock() - starttime5 ) / (double)CLOCKS_PER_SEC );

    // project pixels to 3dPositions and create search structures for the gicp
    projectTo3D(depth, feature_locations_2d_, feature_locations_3d_,pc_col); //takes less than 0.01 sec

    std::clock_t starttime4=std::clock();
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime4) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "projection runtime: " << ( std::clock() - starttime4 ) / (double)CLOCKS_PER_SEC );

    std::clock_t starttime2=std::clock();
    extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime2) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "Feature extraction runtime: " << ( std::clock() - starttime2 ) / (double)CLOCKS_PER_SEC );
    flannIndex = NULL;

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "constructor runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");


}


//void Node::publish2(const char* topic, ros::Time timestamp, Transformation3 transf){
void Node::publish2(const char* topic, ros::Time timestamp, tf::Transform net_transform ){
    ROS_WARN("Sending out a cloud with id %i on topic %s", id_, topic);
    pcl::PointCloud<pcl::PointXYZRGB> cloud ;
    pcl::fromROSMsg (cloudMessage_,cloud);
     boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);
     boost::numeric::ublas::matrix<double> matIn(4, 1);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      //Vector3f tmp(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      //tmp = transf * tmp; //transform to world frame
      //cloud.points[i].x = tmp.x();
      //cloud.points[i].y = tmp.y();
      //cloud.points[i].z = tmp.z();
      double * matrixPtr = matIn.data().begin();

      matrixPtr[0] = cloud.points[i].x;
      matrixPtr[1] = cloud.points[i].y;
      matrixPtr[2] = cloud.points[i].z;
      matrixPtr[3] = 1;
      boost::numeric::ublas::matrix<double> matOut = prod (transform,matIn);
   	  matrixPtr = matOut.data().begin();

      cloud.points[i].x = matrixPtr[0];
      cloud.points[i].y = matrixPtr[1];
      cloud.points[i].z = matrixPtr[2];
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud,msg);
    msg.header.frame_id = topic;
    msg.header.stamp = timestamp;
    cloud_pub2.publish(msg);
    ROS_WARN("ZZZ Pointcloud with id %i sent with frame %s", id_, topic);
}


void Node::publish(const char* topic, ros::Time timestamp){
    ROS_WARN("Sending out a cloud with id %i on topic %s", id_, topic);
    cloudMessage_.header.frame_id = topic;
    cloudMessage_.header.stamp = timestamp;
    cloud_pub.publish(cloudMessage_);
    ROS_WARN("ZZZ Pointcloud with id %i sent with frame %s", id_, topic);
}

// build search structure for descriptor matching
void Node::buildFlannIndex() {

    std::clock_t starttime=std::clock();

    // use same type as in http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
    flannIndex = new cv_flannIndex(feature_descriptors_,
            cv::flann::KDTreeIndexParams(4));
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

int Node::findPairsFlann(Node& other, vector<cv::DMatch>* matches) {

    std::clock_t starttime=std::clock();

    assert(matches->size()==0);

    if (other.flannIndex == NULL) {
        ROS_FATAL("Node::findpairs: flann Index was not initialized");
        return -1;
    }

    // number of neighbours found (has to be two, see l. 57)
    int k = 2;

    // compare
    // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
    cv::Mat indices(feature_descriptors_.rows, k, CV_32S);
    cv::Mat dists(feature_descriptors_.rows, k, CV_32F);


    // get the best two neighbours
    other.flannIndex->knnSearch(feature_descriptors_, indices, dists, k,
            cv::flann::SearchParams(64));

    int* indices_ptr = indices.ptr<int> (0);
    float* dists_ptr = dists.ptr<float> (0);

    cv::DMatch match;
    for (int i = 0; i < indices.rows; ++i) {

        if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1] && abs(feature_locations_2d_[i].pt.y-other.feature_locations_2d_[indices_ptr[2 * i]].pt.y)<10000000) {
        std::cerr<<feature_locations_2d_[i].pt.y<<","<<other.feature_locations_2d_[indices_ptr[2 * i]].pt.y <<endl;
            match.queryIdx = i;
            match.trainIdx = indices_ptr[2 * i];
            match.distance = dists_ptr[2 * i];

            matches->push_back(match);
        }
    }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

    return matches->size();
}

/** remove invalid keypoints (NaN or outside the image) and return the backprojection of valid ones*/
/** Assumptions: No 3d Info available yet */
void Node::projectTo3D(const cv::Mat& depth,
        std::vector<cv::KeyPoint>& feature_locations_2d,
        std::vector<Eigen::Vector4f>& feature_locations_3d,
        const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud){ //TODO: Use this instead of member "pc" or remove argument

    std::clock_t starttime=std::clock();

    cv::Point2f p2d;


    if(feature_locations_3d.size()){
        ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
        feature_locations_3d.clear();
    }


    for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
        p2d = feature_locations_2d[i].pt;
        if (p2d.x >= depth.cols || p2d.x < 0 ||  p2d.y >= depth.rows || p2d.y < 0 ||
                std::isnan(p2d.x) || std::isnan(p2d.y)){
            ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
            feature_locations_2d.erase(feature_locations_2d.begin()+i);
            continue;
        }

        float depth_val = depth.at<float>((int)p2d.y, (int)p2d.x);
        if(std::isnan(depth_val) )  {//No 3d pose would be available
            ROS_DEBUG_STREAM("No depth for: " << p2d);
            feature_locations_2d.erase(feature_locations_2d.begin()+i);
            continue;
        }

        pcl::PointXYZRGB p3d = point_cloud.at((int) p2d.x,(int) p2d.y);

        // Todo: should not happen
        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z)){
            // ROS_INFO("nan in pointcloud! %f, %f",p2d.x, p2d.y);
            feature_locations_2d.erase(feature_locations_2d.begin()+i);
            continue;
        }


        feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
        i++; //Only increment if no element is removed from vector
    }



    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

}



void Node::computeInliersAndError(const std::vector<cv::DMatch>& matches,
        const Eigen::Matrix4f& transformation,
        const std::vector<Eigen::Vector4f>& origins,
        const std::vector<Eigen::Vector4f>& earlier,
        std::vector<cv::DMatch>& inliers, //output var
        double& mean_error,vector<double>& errors,
        double squaredMaxInlierDistInM) const{ //output var

    std::clock_t starttime=std::clock();

    inliers.clear();

    vector<pair<float,int> > dists;
    std::vector<cv::DMatch> inliers_temp;

    mean_error = 0.0;
    for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

        unsigned int this_id = matches[j].queryIdx;
        unsigned int earlier_id = matches[j].trainIdx;

        Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];
        double error = vec.dot(vec);

        if(error > squaredMaxInlierDistInM)
            continue; //ignore outliers


        error = sqrt(error);
        dists.push_back(pair<float,int>(error,j));
        inliers_temp.push_back(matches[j]); //include inlier

        mean_error += error;
        errors.push_back(error);
    }

    if (inliers_temp.size()==0){
        mean_error = -1;
        inliers = inliers_temp;
    }
    else
    {
        mean_error /= inliers_temp.size();

        // sort inlier ascending according to their error
        sort(dists.begin(),dists.end());

        inliers.resize(inliers_temp.size());
        for (unsigned int i=0; i<inliers_temp.size(); i++){
            inliers[i] = matches[dists[i].second];
        }
    }


    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

}

///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node& earlier_node,
        std::vector<cv::DMatch>* initial_matches,
        Eigen::Matrix4f& resulting_transformation,
        float& rmse,
        std::vector<cv::DMatch>& matches, //for visualization?
        unsigned int min_inlier_threshold,
        unsigned int ransac_iterations) const{

    std::clock_t starttime=std::clock();

    assert(initial_matches != NULL);



    matches.clear();
    std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
    if(initial_matches->size() <= min_inlier_threshold){
        ROS_DEBUG("Only %d feature matches between frames %d and %d",(int)initial_matches->size() , this->id_, earlier_node.id_);
        return false;
    }

    double inlier_error; //all squared errors
    srand((long)std::clock());
    pcl::TransformationFromCorrespondences tfc;

    // a point is an inlier if it's no more than max_dist_m m from its partner apart
    float max_dist_m = 0.02;

    unsigned int n_iter = 0;

    vector<double> errors;
    vector<double> temp_errorsA;

    double best_error = 1e6;

    int valid_iterations = 0;

    unsigned int good_inlier_cnt = min(20,int(initial_matches->size()*0.2));

    for (; n_iter < ransac_iterations; n_iter++) {
        tfc.reset();

        // chose randomly from the correspondences:
        int sample_size = 3;

        // TODO: assert that no point is drawn twice
        for (int k = 0; k < sample_size; k++) {
            int id = rand() % initial_matches->size();

            int this_id = initial_matches->at(id).queryIdx;
            int earlier_id = initial_matches->at(id).trainIdx;

            Eigen::Vector3f from(this->feature_locations_3d_[this_id][0],
                    this->feature_locations_3d_[this_id][1],
                    this->feature_locations_3d_[this_id][2]);
            Eigen::Vector3f  to (earlier_node.feature_locations_3d_[earlier_id][0],
                    earlier_node.feature_locations_3d_[earlier_id][1],
                    earlier_node.feature_locations_3d_[earlier_id][2]);
            tfc.add(from, to);
        }

        // get relative movement from samples
        Eigen::Matrix4f transformation = tfc.getTransformation().matrix();

        computeInliersAndError(*initial_matches, transformation,
                this->feature_locations_3d_,
                earlier_node.feature_locations_3d_,
                inlier, inlier_error, temp_errorsA, max_dist_m*max_dist_m);


        if (inlier.size() < good_inlier_cnt || inlier_error > max_dist_m)
            continue;
        ///Iterations with more than half of the initial_matches inlying, count twice
        if (inlier.size() > initial_matches->size()*0.5)
            n_iter++;
        ///Iterations with more than 80% of the initial_matches inlying, count threefold
        if (inlier.size() > initial_matches->size()*0.8)
            n_iter++;


        valid_iterations++;

        assert(inlier_error>0);

        if (inlier_error < best_error  )
        {

            resulting_transformation = transformation;
            matches = inlier;
            assert(matches.size()>= good_inlier_cnt);

            rmse = inlier_error;
            errors = temp_errorsA;

            best_error = inlier_error;
        }


        int max_ndx = min((int) good_inlier_cnt,30);

        double new_inlier_error;

        // compute new trafo from inliers:
        for (int k = 0; k < sample_size; k++) {
            int id = rand() % max_ndx;

            int this_id = inlier[id].queryIdx;
            int earlier_id = inlier[id].trainIdx;

            Eigen::Vector3f from(this->feature_locations_3d_[this_id][0],
                    this->feature_locations_3d_[this_id][1],
                    this->feature_locations_3d_[this_id][2]);
            Eigen::Vector3f  to (earlier_node.feature_locations_3d_[earlier_id][0],
                    earlier_node.feature_locations_3d_[earlier_id][1],
                    earlier_node.feature_locations_3d_[earlier_id][2]);
            tfc.add(from, to);
        }

        // get relative movement from samples
        transformation = tfc.getTransformation().matrix();

        computeInliersAndError(*initial_matches, transformation,
                this->feature_locations_3d_,
                earlier_node.feature_locations_3d_,
                inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);

        if (inlier.size() < good_inlier_cnt || new_inlier_error > max_dist_m)
            continue;

        assert(new_inlier_error>0);

        if (new_inlier_error < best_error  )
        {
            resulting_transformation = transformation;
            matches = inlier;

            assert(matches.size()>= good_inlier_cnt);
            rmse = new_inlier_error;
            errors = temp_errorsA;

            best_error = new_inlier_error;
        }

    } //iterations


    ROS_DEBUG("asd %i good iterations, inlier percentage %i, inlier cnt: %i ",valid_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size());
    ROS_DEBUG("asd time: %.2f, error: %.3f cm",( std::clock() - starttime ) / (double)CLOCKS_PER_SEC, rmse*100 );

    for (unsigned int i=0; i < errors.size(); i++){
        // ROS_INFO("asd error(%i) = %.4f cm",i,errors.at(i)*100);
    }


    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.01, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");


    return (matches.size() > 0);
}




