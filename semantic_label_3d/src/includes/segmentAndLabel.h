

#include <stdint.h>

#include "color.cpp"
#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/io/pcd_io.h"
#include "point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "CombineUtils.h"

typedef pcl::PointXYZRGBCamSL PointT;
typedef pcl::PointXYGRGBCam PointCamT;
typedef pcl::PointXYZRGB PointFrameT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;

void apply_segment_filter_frame(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment) {
          j++;
          outcloud.points[j].x = incloud.points[i].x;
          outcloud.points[j].y = incloud.points[i].y;
          outcloud.points[j].z = incloud.points[i].z;
          outcloud.points[j].rgb = incloud.points[i].rgb;
          outcloud.points[j].segment = incloud.points[i].segment;
          outcloud.points[j].label = incloud.points[i].label;
          outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
          outcloud.points[j].distance = incloud.points[i].distance;

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }

   // cout<<j << ","<<segment<<endl;
    if(j>=0)
        outcloud.points.resize ( j+1 );
    else
       outcloud.points.clear ();
}

const int AMBIGOUS_LABEL=125; 
int getMajorityLabel(const pcl::PointCloud<PointT> &cloud){
    map<int,int> label_count_map;
    int max_count=0;
    int max_label=0;
    int second_max_label=0;
    int second_max_count =0;

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        label_count_map[cloud.points[i].label] ++;
    }
    multimap<int,int> count_label_map;
    for (map<int, int>::iterator it= label_count_map.begin(); it != label_count_map.end(); it++)
    {
        if((*it).first!=0&&(*it).first!=8 && (*it).first!=19&& (*it).first!=17)
        count_label_map.insert(pair<int,int>((*it).second,(*it).first));
    }
    multimap<int, int>::reverse_iterator it = count_label_map.rbegin();
    if (it != count_label_map.rend()) {
        max_count = (*it).first;
        max_label = (*it).second;
        it++;

        if (it != count_label_map.rend()) {
            second_max_count = (*it).first;
            second_max_label = (*it).second;
        }
    }
    //cout << "max_count:" << max_count << " second_max_count:" << second_max_count << " segment_size:" << cloud.points.size() << endl;
    if (max_count > 20 )
  //  if (max_count > cloud.points.size()/10 )
    {
        if( (max_count - second_max_count)> max_count/2 )
             return max_label;
        else
        {
            cerr<<max_label<<":"<<max_count<<","<<second_max_label<<":"<<second_max_count<<",";
            return AMBIGOUS_LABEL;//ambigous label
        }
    }
    return 0;
}

void findConsistentLabels (const pcl::PointCloud<PointT> &incloud,   pcl::PointCloud<PointT> &outcloud) {
    pcl::PointCloud<PointT>::Ptr incloud_ptr(new pcl::PointCloud<PointT > (incloud));
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());

    map<int,int> segment_label_map;


    // find the segment list
    map<int, int> segments;
    for (size_t i = 0; i < incloud.points.size(); ++i) {
        if (incloud.points[i].label != 0)
            segments[(int) incloud.points[i].segment] = 1;
    }
    // for each segment
    for (map<int, int>::iterator it2 = segments.begin(); it2 != segments.end(); it2++) {

        int segnum = (*it2).first;
        // filter segment
        apply_segment_filter_frame(*incloud_ptr, *cloud_seg, segnum);
        // find majority label segment

        segment_label_map[segnum] = getMajorityLabel(*cloud_seg);
     //   cerr<<"segment "<<segnum<<" is "<<segment_label_map[segnum]<<endl;
        if(segment_label_map[segnum]==AMBIGOUS_LABEL)
            cerr<<"segment "<<segnum<<" is ambigous"<<endl;
    }

    segment_label_map[0] = 0;


    // apply segment label map to all points in the cloud
    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points.resize ( incloud.points.size() );
    for (size_t i = 0; i < incloud.points.size(); i++) {
        outcloud.points[i].x = incloud.points[i].x;
        outcloud.points[i].y = incloud.points[i].y;
        outcloud.points[i].z = incloud.points[i].z;
        outcloud.points[i].rgb = incloud.points[i].rgb;
        outcloud.points[i].cameraIndex = incloud.points[i].cameraIndex;
        outcloud.points[i].distance = incloud.points[i].distance;
        outcloud.points[i].segment = incloud.points[i].segment;
        outcloud.points[i].label = segment_label_map[incloud.points[i].segment];
    }


}


void findlabel(const pcl::PointCloud<PointT> &cloud, const  pcl::PointCloud<PointFrameT> &cloudframe , pcl::PointCloud<PointT> &outcloud,VectorG origin,int camIndex) {

    std::vector<int> nn_indices;
    nn_indices.resize(2);
    std::vector<float> nn_distances;
    nn_distances.resize(2);
    float threshold = 0.005;

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    outcloud.header.frame_id = cloud.header.frame_id;
    outcloud.points.resize(cloudframe.points.size());
    for (size_t i = 0; i < cloudframe.points.size(); i++) {
        outcloud.points[i].x = cloudframe.points[i].x;
        outcloud.points[i].y = cloudframe.points[i].y;
        outcloud.points[i].z = cloudframe.points[i].z;
        outcloud.points[i].rgb = cloudframe.points[i].rgb;
        outcloud.points[i].cameraIndex = camIndex;
        if(isnan(outcloud.points[i].x))
            outcloud.points[i].distance=0;
        else            
            outcloud.points[i].distance = (origin.subtract(VectorG(outcloud.points[i].x,outcloud.points[i].y,outcloud.points[i].z))).getNorm();
        
        outcloud.points[i].segment = 0;
        outcloud.points[i].label = 0;
    }


    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
    tree->setInputCloud(cloud_ptr);
    int counter =0;
    for (size_t i = 0; i < outcloud.points.size(); ++i) {

        //if (!tree->radiusSearch ((*small_cloud).points[i], tolerance, nn_indices, nn_distances))
        tree->nearestKSearch(outcloud.points[i], 2, nn_indices, nn_distances);
        if (nn_distances[1] < threshold) {
            counter++;
            outcloud.points[i].label = cloud.points[nn_indices[1]].label;
        }
    }
    cout << "cloud size:" << outcloud.points.size() << " found " << counter << " point labels" << endl;

}





/* 
Extract the clusters based on location and normals
*/
void extractEuclideanClusters (
      const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
      const boost::shared_ptr<pcl::KdTree<PointT> > &tree,
      float tolerance, std::vector<pcl::PointIndices> &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    float adjTolerance = 0;
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!", cloud.points.size (), normals.points.size ());
      return;
    }
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (processed[i])
        continue;

      std::vector<int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (i);
      processed[i] = true;

      int cnt = 0;
 
      while (sq_idx < (int)seed_queue.size ())
      { 
         cnt++;
         //ROS_INFO ("i = %d, cnt = %d", i , cnt);

        // Search for sq_idx
        adjTolerance = cloud.points[seed_queue[sq_idx]].distance * tolerance;
        //adjTolerance = tolerance;
        if (!tree->radiusSearch (seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                             // Has this point been processed before ?
            continue;

          processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p =
            normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
            normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
            normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle )
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }

        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
      {
        pcl::PointIndices r;
        r.indices.resize (seed_queue.size ());
        for (size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        //ROS_INFO ("cluster of size %d data point\n ",r.indices.size());
        clusters.push_back (r);
      }
    }
  }





void getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud,
                const std::vector<pcl::PointIndices> &clusters2,
                pcl::PointCloud<PointT> &combined_cloud )
{
    
  combined_cloud.header.frame_id = cloud.header.frame_id;
  combined_cloud.points.resize(cloud.points.size());
  for (size_t i =0 ; i<cloud.points.size(); i++)
  {
      combined_cloud.points[i].x = cloud.points[i].x;
      combined_cloud.points[i].y = cloud.points[i].y;
      combined_cloud.points[i].z = cloud.points[i].z;
      combined_cloud.points[i].rgb = cloud.points[i].rgb;
      combined_cloud.points[i].cameraIndex = cloud.points[i].cameraIndex;
      combined_cloud.points[i].distance = cloud.points[i].distance;
      combined_cloud.points[i].segment = 0;
      combined_cloud.points[i].label = cloud.points[i].label;
  }

  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    for (size_t j = 0; j < clusters2[i].indices.size (); ++j)
    {
      combined_cloud.points[clusters2[i].indices[j]].segment = i;
    }
  }
}

void getClustersFromPointCloud2InPlace (pcl::PointCloud<PointT> &cloud,
                const std::vector<pcl::PointIndices> &clusters2)
{    
  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    for (size_t j = 0; j < clusters2[i].indices.size (); ++j)
    {
      cloud.points[clusters2[i].indices[j]].segment = i;
    }
  }
}

void segment (const pcl::PointCloud<PointT> &cloud,  pcl::PointCloud<PointT> &outcloud){

    int min_pts_per_cluster = 1500;
    int max_pts_per_cluster = INT_MAX;//3000000;
    assert(max_pts_per_cluster>3000000); //no overflows!
    
    int number_neighbours = 50;
    float radius = 0.01;// 0.025
    float angle = 0.52;
    KdTreePtr normals_tree_, clusters_tree_;
    pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
    std::vector<pcl::PointIndices> clusters;


    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    initTree (0, clusters_tree_);
    clusters_tree_->setInputCloud (cloud_ptr);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    n3d_.setKSearch (number_neighbours);
    n3d_.setSearchMethod (normals_tree_);

    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(cloud_normals);
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);

    extractEuclideanClusters ( cloud, *cloud_normals_ptr, clusters_tree_, radius, clusters, angle, min_pts_per_cluster, max_pts_per_cluster);
    ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

    getClustersFromPointCloud2(*cloud_ptr, clusters, outcloud);

}

void segmentInPlace (pcl::PointCloud<PointT> &cloud,int min_pts_per_cluster = 1000){

    int max_pts_per_cluster = INT_MAX;//3000000;
    assert(max_pts_per_cluster>3000000); //no overflows!
    
    int number_neighbours = 50;
    float radius = 0.02;// 0.025
    float angle = 0.52;
    KdTreePtr normals_tree_, clusters_tree_;
    pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
    std::vector<pcl::PointIndices> clusters;


    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    initTree (0, clusters_tree_);
    clusters_tree_->setInputCloud (cloud_ptr);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    n3d_.setKSearch (number_neighbours);
    n3d_.setSearchMethod (normals_tree_);

    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(cloud_normals);
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);

    extractEuclideanClusters ( cloud, *cloud_normals_ptr, clusters_tree_, radius, clusters, angle, min_pts_per_cluster, max_pts_per_cluster);
    ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

    getClustersFromPointCloud2InPlace(cloud, clusters);

}

void convertType(const pcl::PointCloud<PointFrameT> &cloud,  pcl::PointCloud<PointT> &outcloud,VectorG origin,int camIndex){

    cout<<"originc: "<<origin.v[0]<<" "<<origin.v[1]<<" "<<origin.v[2]<<endl;
  outcloud.header.frame_id = cloud.header.frame_id;
  outcloud.points.resize(cloud.points.size());
  for (size_t i =0 ; i<cloud.points.size(); i++)
  {
      outcloud.points[i].x = cloud.points[i].x;
      outcloud.points[i].y = cloud.points[i].y;
      outcloud.points[i].z = cloud.points[i].z;
      outcloud.points[i].rgb = cloud.points[i].rgb;
      outcloud.points[i].cameraIndex = camIndex;
        if(isnan(outcloud.points[i].x))
            outcloud.points[i].distance=0;
        else            
            outcloud.points[i].distance = (origin.subtract(VectorG(outcloud.points[i].x,outcloud.points[i].y,outcloud.points[i].z))).getNorm();
      outcloud.points[i].segment = 0;
      outcloud.points[i].label = 0;
  }
}

/*void segment (const pcl::PointCloud<PointFrameT> &cloud,  pcl::PointCloud<PointT> &outcloud){

    pcl::PointCloud<PointT> newcloud;
    convertType(cloud,newcloud,origin,camIndex);
    segment(newcloud,outcloud);

}*/



/* ---[ 
int
  main (int argc, char** argv)
{


  sensor_msgs::PointCloud2 cloud_blob;

  pcl::PointCloud<PointCamT> cloud;

  pcl::PCDWriter writer;

 // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s  with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1], pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blob, cloud);
   pcl::PointCloud<PointCamT>::Ptr cloud_ptr (new pcl::PointCloud<PointCamT> (cloud));
  
   pcl::PointCloud<PointT> cloud_segmented;
   segment (cloud, cloud_segmented);

 
  // extractEuclideanClusters ( *cloud_filtered, *cloud_normals_ptr, clusters_tree_, radius, hue_tolerance, clusters,angle, min_pts_per_cluster, max_pts_per_cluster);
  


  
  

  std::string fn (argv[1]);
  fn = fn.substr(0,fn.find('.'));

  fn = fn + "_segmented_xyzn.pcd";
  writer.write ( fn,cloud_segmented, true);

cout <<"wrote file "<<fn<<endl;
  return (0);
}
 ]--- */
