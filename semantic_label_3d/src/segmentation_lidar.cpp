

#include <stdint.h>

#include "includes/color.cpp"
#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

typedef pcl::PointXYZRGBCamSL PointT;
typedef pcl::PointXYZRGBCamSL PointOutT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;



void extractEuclideanClusters (const pcl::PointCloud<PointT> &cloud,
                               const boost::shared_ptr<KdTree > &tree,
                               float tolerance, std::vector<pcl::PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)

{
  
     // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
  //and indices[i]
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud ()->points.size (), cloud.points.size ());
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

    int cnt=0;

    while (sq_idx < (int)seed_queue.size ())
    {
      cnt++;
      //ROS_INFO ("i = %d, cnt = %d",i,cnt);
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      {
       
        if (processed[nn_indices[j]])                             // Has this point been processed before ?
          continue;

        // Perform a simple Euclidean clustering
        seed_queue.push_back (nn_indices[j]);
        processed[nn_indices[j]] = true;
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        // This is the only place where indices come into play
        r.indices[j] = seed_queue[j];

      //r.indices.assign(seed_queue.begin(), seed_queue.end());
      sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }

}

void extractEuclideanClusters (
      const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
      const std::vector<int> &indices, const boost::shared_ptr<KdTree > &tree,
      float tolerance, std::vector<pcl::PointIndices> &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (tree->getIndices ()->size () != indices.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%zu) than the input set (%zu)!", tree->getIndices ()->size (), indices.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!", cloud.points.size (), normals.points.size ());
      return;
    }
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (indices.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < indices.size (); ++i)
    {
      if (processed[i])
        continue;

      std::vector<int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (i);
    processed[i] = true;

      while (sq_idx < (int)seed_queue.size ())
      { 
        // Search for sq_idx
        if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
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
            normals.points[indices[i]].normal[0] * normals.points[indices[nn_indices[j]]].normal[0] +
            normals.points[indices[i]].normal[1] * normals.points[indices[nn_indices[j]]].normal[1] +
            normals.points[indices[i]].normal[2] * normals.points[indices[nn_indices[j]]].normal[2];
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
          r.indices[j] = indices[seed_queue[j]];

        sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);
      }
    }
  }



/* 
Extract the clusters based on location and normals
*/
void extractEuclideanClusters (
      const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
      const boost::shared_ptr<KdTree > &tree,
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
        //adjTolerance = cloud.points[seed_queue[sq_idx]].distance * tolerance;
        adjTolerance = tolerance;
        if (!tree->radiusSearch (seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); j++)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                             // Has this point been processed before ?
            continue;

          //processed[nn_indices[j]] = true;
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
        for (size_t j = 0; j < seed_queue.size (); j++)
          r.indices[j] = seed_queue[j];

        sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        //ROS_INFO ("cluster of size %d data point\n ",r.indices.size());
        clusters.push_back (r);
      }
    }
  }


/* 
Extract the clusters based on location,normals and color
*/
void extractEuclideanClusters (
      const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
      const boost::shared_ptr<KdTree > &tree,
      float tolerance, float rgb_tolerance, std::vector<pcl::PointIndices> &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    float adjTolerance = 0;
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
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
        //adjTolerance = cloud.points[seed_queue[sq_idx]].distance * tolerance;
        adjTolerance =  tolerance;
        if (!tree->radiusSearch (seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                             // Has this point been processed before ?
            continue;

      //    processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p =
            normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
            normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
            normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
          ColorRGB a (cloud.points[i].rgb);
		  ColorRGB b (cloud.points[nn_indices[j]].rgb);
          double color_diff = ColorRGB::HSVdistance(a, b);
//ROS_INFO ("diff = %f",color_diff);
          if ( fabs (acos (dot_p)) < eps_angle && color_diff < rgb_tolerance )
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




void getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud_objects,
                const std::vector<pcl::PointIndices> &clusters2,
                //std::vector<pcl::PointCloud<my_ns::MyPoint> > &clusters,
                std::vector<pcl::PointCloud<PointOutT> > &clusters,
                //pcl::PointCloud<my_ns::MyPoint> &combined_cloud )
                pcl::PointCloud<PointOutT> &combined_cloud )
{
  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); i++)
  {
    clusters[i].header.frame_id = cloud_objects.header.frame_id;
  //  clusters[i].header.stamp = ros::Time(0);
    clusters[i].points.resize (clusters2[i].indices.size ());
    for (size_t j = 0; j < clusters[i].points.size (); j++)
    {
      clusters[i].points[j].x = cloud_objects.points[clusters2[i].indices[j]].x;
      clusters[i].points[j].y = cloud_objects.points[clusters2[i].indices[j]].y;
      clusters[i].points[j].z = cloud_objects.points[clusters2[i].indices[j]].z;
      clusters[i].points[j].rgb = cloud_objects.points[clusters2[i].indices[j]].rgb;
      clusters[i].points[j].cameraIndex = cloud_objects.points[clusters2[i].indices[j]].cameraIndex;
      clusters[i].points[j].distance = cloud_objects.points[clusters2[i].indices[j]].distance;
      clusters[i].points[j].segment = i;
      clusters[i].points[j].label = cloud_objects.points[clusters2[i].indices[j]].label;
    }
    
    if(i == 0) {combined_cloud = clusters[i];}
    else { combined_cloud += clusters[i]; }
  }
}


/* ---[ */
int
  main (int argc, char** argv)
{

  int min_cluster_size_= 300;
  int min_pts_per_cluster = 0;
  int max_pts_per_cluster = 3000000;
  int number_neighbours = 50;
  float radius = 1.2;// 0.025 
  float angle = 0.52;
  float hue_tolerance = 50;


  sensor_msgs::PointCloud2 cloud_blob;

  pcl::PointCloud<PointT> cloud;
  KdTreePtr normals_tree_, clusters_tree_;

  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::PassThrough<PointT> pass_;
  pcl::NormalEstimation<PointT, pcl::Normal> n3d_;

  // Normal estimation parameters
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
  n3d_.setKSearch (number_neighbours);
  n3d_.setSearchMethod (normals_tree_);

 
 // read from file
  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s  with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), argv[1], pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
   pcl::fromROSMsg (cloud_blob, cloud);
   pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
  
  //Step1: remove nans
  pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT> ());
  pass_.setInputCloud (cloud_ptr);
  pass_.filter (*cloud_filtered);

  if (cloud_filtered->points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered->points.size());
    return 0;
  }
  //Step2 : downsample

//  *cloud_filtered = *cloud_ptr; 
  

  //Step3: find the normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d_.setInputCloud (cloud_filtered);
  n3d_.compute (cloud_normals);
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr =  boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
  ROS_INFO("Step 2 done");

  
  // Cluster the points
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
  std::vector<pcl::PointIndices> clusters;
  

  ROS_INFO("here2");


  std::vector<int> indices (cloud_filtered->points.size ());
  for (size_t i = 0; i < cloud.points.size (); ++i)
  { 
    indices.push_back(i);
  }
  boost::shared_ptr <const std::vector<int> > indicesp (new std::vector<int>(indices));
  //pcl::PointIndices::Ptr indicesp (new pcl::PointIndices( ));
  initTree (0, clusters_tree_);
  clusters_tree_->setInputCloud (cloud_filtered );// ,indicesp);
 // extractEuclideanClusters ( *cloud_filtered, clusters_tree_, tolerance, clusters,  min_pts_per_cluster, max_pts_per_cluster);
 
  if (atoi(argv[2]) == 1)
   extractEuclideanClusters ( *cloud_filtered, *cloud_normals_ptr, clusters_tree_, radius, clusters, angle, min_pts_per_cluster, max_pts_per_cluster);
  else
   extractEuclideanClusters ( *cloud_filtered, *cloud_normals_ptr, clusters_tree_, radius, hue_tolerance, clusters,angle, min_pts_per_cluster, max_pts_per_cluster);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

  //std::vector<pcl::PointCloud<my_ns::MyPoint> > clusters2;
  //pcl::PointCloud<my_ns::MyPoint> combined_cloud;
  std::vector<pcl::PointCloud<PointOutT> > clusters2;
  pcl::PointCloud<PointOutT> combined_cloud;
  getClustersFromPointCloud2(*cloud_filtered, clusters, clusters2,combined_cloud);
/*  for (size_t i = 0; i< clusters2.size(); i++)
  {
    std::stringstream ss;
    ss << i;
    std::string fn = "cluster"+ ss.str() + ".pcd";

    writer.write ( fn, clusters2[i], false);

  }*/
  std::string fn (argv[1]);
  fn = fn.substr(0,fn.find('.'));

  if (atoi(argv[2]) == 1) {fn = fn + "_segmented_xyzn.pcd";} else { fn = fn + "_segmented_xyzrgbn.pcd";}
  writer.write ( fn,combined_cloud, true);

cout <<"wrote file "<<fn<<endl;
  return (0);
}
/* ]--- */
