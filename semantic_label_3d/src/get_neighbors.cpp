#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "descriptors_3d/all_descriptors.h"
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
//typedef pcl::PointXYGRGBCam PointT;
typedef pcl::PointXYZRGBCamSL PointT;

using namespace pcl;

float getSmallestDistance (const pcl::PointCloud<PointT> &cloud1,const pcl::PointCloud<PointT> &cloud2)
{
  float min_distance = FLT_MAX;
  for (size_t i = 0; i < cloud1.points.size (); ++i)
  { 
   for (size_t j = 0; j < cloud2.points.size (); ++j)
   {
      float distance = pow(cloud1.points[i].x - cloud2.points[j].x,2) + pow(cloud1.points[i].y - cloud2.points[j].y,2) + pow(cloud1.points[i].z - cloud2.points[j].z,2);
     // cerr<< "i,j = " << i << "," << j<< " dist = " <<distance << endl;
      if (min_distance > distance) min_distance = distance;
   }
  }
  return sqrt(min_distance) ;
}


void get_neighbors ( const std::vector<pcl::PointCloud<PointT> > &segment_clouds, map< pair <int,int> , float > &distance_matrix, map <int , vector <int> > &neighbor_map )
{

// get distance matrix
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
      for (size_t j = i+1; j < segment_clouds.size() ; j++)
      { 
         float dist = getSmallestDistance(segment_clouds[i],segment_clouds[j]);
         distance_matrix[make_pair(segment_clouds[i].points[1].segment,segment_clouds[j].points[1].segment)] = dist;
         distance_matrix[make_pair(segment_clouds[j].points[1].segment,segment_clouds[i].points[1].segment)] = dist;
      }
 //     std::cerr<< "size of segment " << i << " : " << segment_clouds[i].points.size() << "\t and label is: " << segment_clouds[i].points[1].label <<"\n";
    }
// get neighbour map
    for ( map< pair <int,int> , float >::iterator it=distance_matrix.begin() ; it != distance_matrix.end(); it++ )
    {   
      if((*it).second < 0.1)  neighbor_map[(*it).first.first].push_back((*it).first.second);
 //     cout << (*it).first.first << "," << (*it).first.second <<" => " << (*it).second << endl;
    }
/*
// printing the neighbor_map
    for ( map< int, vector <int> >::iterator it=neighbor_map.begin() ; it != neighbor_map.end(); it++ )
    {
      cout << (*it).first << " => ";
      for (vector<int>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++)
        cout << "," << (*it2) ;
      cout << endl;
    }*/
}

int main(int argc, char** argv) {

    sensor_msgs::PointCloud2 cloud_blob;
   
    pcl::PointCloud<PointT> cloud;
    std::ofstream outfile;
    outfile.open("matrix.txt",ios::app);

    // read the pcd file

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

    // convert to templated message type

    pcl::fromROSMsg(cloud_blob, cloud);

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    //pcl::PointCloud<PointXYZI>::Ptr cloud_seg (new pcl::PointCloud<PointXYZI> ());
    std::vector<pcl::PointCloud<PointT> > segment_clouds;
    std::map<int,int>  segment_num_index_map;
    pcl::PointIndices::Ptr segment_indices(new pcl::PointIndices());

    // get segments

    // find the max segment number
    int max_segment_num = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (max_segment_num < cloud.points[i].segment) {
            max_segment_num = (int) cloud.points[i].segment;
        }
    }

    ExtractIndices<PointT> extract;

    int index_ = 0;
    for (int seg = 1; seg <= max_segment_num; seg++) {
        vector<float> features;
        int label;
        segment_indices->indices.clear();
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            if (cloud.points[i].segment == seg) {
                segment_indices->indices.push_back(i);
                label = cloud.points[i].label;
            }
        }
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(segment_indices);
        extract.setNegative(false);
        extract.filter(*cloud_seg);
       //  std::cerr << seg << ". Cloud size after extracting : " << cloud_seg->points.size() << std::endl;
        if (cloud_seg->points.size() > 10 && cloud_seg->points[1].label != 0) {
			segment_clouds.push_back(*cloud_seg);
			segment_num_index_map[cloud_seg->points[1].segment] = index_;
			index_ ++; 
        }
    }
    map< pair <int,int> , float > distance_matrix;
    map <int , vector <int> > neighbor_map;
    get_neighbors ( segment_clouds, distance_matrix, neighbor_map );


    cout << "DONE!!\n";
    // write features to file
/*    for (vector<float>::iterator it = features.begin(); it < features.end(); it++) {
      outfile << *it << "\t";
    }
    outfile << "\n";
 */
    outfile.close();


}


