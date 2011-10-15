#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"
//#include "descriptors_3d/all_descriptors.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
int IM_WIDTH=320;
int IM_HEIGHT=240;
#include "includes/genericUtils.h"
#include <Eigen/Dense>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <point_cloud_mapping/geometry/nearest.h>
#include <pcl_ros/io/bag_io.h>
#include "HOG.cpp"
typedef pcl::PointXYInt PointT;
#include "includes/CombineUtils.h"
#include<boost/numeric/ublas/matrix.hpp>
#include<boost/numeric/ublas/io.hpp>


#include "pcl_visualization/pcl_visualizer.h"
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
#include "time.h"

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace boost;

#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>

using namespace std;
using namespace octomap;
using namespace Eigen;
//#include <Eig>
//typedef pcl::PointXYGRGBCam PointT;

//typedef  pcl::KdTree<PointT> KdTree;
//typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;
bool UseVolFeats=false;
pcl::PCDWriter writer;

using namespace pcl;
ColorRGB getColorOfPixel(IplImage *image, int y,int x)
{
    float r,g,b;
assert(y<image->height);
assert(x<image->width);
        b=CV_IMAGE_ELEM ( image, uchar, y, 3 * x );
        g=CV_IMAGE_ELEM ( image, uchar, y, 3 * x + 1 );
        r=CV_IMAGE_ELEM ( image, uchar, y, 3 * x + 2 );
//cout<<r<<" "<<g<<" "<<b<<endl;
        return ColorRGB(r/255.0,g/255.0,b/255.0);
}
class OriginalFrameInfo
{
  HOG hogDescriptors;
  TransformG cameraTrans;
  bool cameraTransSet;
  
public:
   pcl::PointCloud<pcl::PointXYZRGBCamSL>::ConstPtr RGBDSlamFrame; // required to get 2D pixel positions
  
  OriginalFrameInfo(IplImage *image)
  {
    cameraTransSet=false;
          hogDescriptors.computeHog (image);
          
  }
  
  
  static void findHog(pcl::PointCloud<PointT> &incloud, HOGFeaturesOfBlock &hogSegment, OriginalFrameInfo*  targetFrame)
  {
    
    vector<Point2DAbhishek> pointsInImageLyingOnSegment;
    for(size_t i=0;i<incloud.size ();i++)
      {
              pointsInImageLyingOnSegment.push_back (Point2DAbhishek(incloud.points[i].x,incloud.points[i].y));     
      }
    
    targetFrame->hogDescriptors.getFeatValForPixels (pointsInImageLyingOnSegment,hogSegment);
   // targetFrame->saveImage (incloud.points[pointIndices[1]].segment,incloud.points[pointIndices[1]].label,pointsInImageLyingOnSegment);
    
  }

  void
  setCameraTrans (TransformG cameraTrans)
  {
    this->cameraTrans = cameraTrans;
    cameraTransSet=true;
  }
  
  void
  applyPostGlobalTrans (TransformG globalTrans)
  {
    //post => premultiply coz point is towards right;
    if(cameraTransSet)
    cameraTrans=cameraTrans.preMultiply (globalTrans);
  }

  TransformG
  getCameraTrans () const
  {
    assert(cameraTransSet);
    return cameraTrans;
  }

  void
  setCameraTransSet (bool cameraTransSet)
  {
    this->cameraTransSet = cameraTransSet;
  }

  bool
  isCameraTransSet () const
  {
    return cameraTransSet;
  }

  bool
  isEmpty () const
  {
    return RGBDSlamFrame->size()==0;
  }
  
};


OriginalFrameInfo * originalFrame;

class SpectralProfile
{
  vector<float> eigenValues; // sorted in ascending order
public:
      pcl::PointCloud<PointT>::Ptr cloudPtr ;
  HOGFeaturesOfBlock avgHOGFeatsOfSegment;
  float avgH;
  float avgS;
  float avgV;
  
  geometry_msgs::Point32 centroid;
  Eigen::Vector3d normal;
  void setEigValues(Eigen::Vector3d eigenValues_)
  {
    eigenValues.clear ();
    //Assuming the values are sorted
    assert(eigenValues_(0)<=eigenValues_(1));
    assert(eigenValues_(1)<=eigenValues_(2));
            
    for(int i=0;i<3;i++)
      eigenValues.push_back (eigenValues_(i));
  //  std::sort (eigenValues.begin (),eigenValues.end ()); // sorted in ascending order
  }
  
  float getDescendingLambda(int index) const
  {
    return eigenValues[2-index];
  }
  
  float getScatter() const
  {
    return getDescendingLambda (0);
  }
  
  float getLinearNess() const
  {
    return (getDescendingLambda (0)-getDescendingLambda (1));
  }
  
  float getPlanarNess() const
  {
    return (getDescendingLambda (1)-getDescendingLambda (2));
  }
  
  float getNormalZComponent() const
  {
    return normal[2];
  }
  
  float getAngleWithVerticalInRadians() const
  {
    return acos(getNormalZComponent());
  }
  
  float getHorzDistanceBwCentroids(const SpectralProfile & other) const
  {
     return sqrt(pow(centroid.x - other.centroid.x, 2) + pow(centroid.y - other.centroid.y, 2));    
  }
  
  float getVertDispCentroids(const SpectralProfile & other)
  {
     return (centroid.z - other.centroid.z);    
  }
  
  float getHDiffAbs(const SpectralProfile & other)
  {
     return fabs(avgH - other.avgH);    
  }
  
  float getSDiff(const SpectralProfile & other)
  {
     return (avgS - other.avgS);    
  }
  
  float getVDiff(const SpectralProfile & other)
  {
     return (avgV - other.avgV);    
  }
  
  float getAngleDiffInRadians(const SpectralProfile & other)
  {
        return (getAngleWithVerticalInRadians() - other.getAngleWithVerticalInRadians ());  
  }
  
  float getNormalDotProduct(const SpectralProfile & other)
  {
        return  fabs(normal(0)*other.normal(0) +normal(1)*other.normal(1) +normal(2)*other.normal(2)) ;
  }
  
  float getInnerness(const SpectralProfile & other)
  {
    float r1=sqrt(centroid.x*centroid.x+centroid.y*centroid.y);
    float r2=sqrt(other.centroid.x*other.centroid.x+other.centroid.y*other.centroid.y);
    return r1-r2;
  }

  float pushHogDiffFeats(const SpectralProfile & other, vector<float> & feats)
  {
    avgHOGFeatsOfSegment.pushBackAllDiffFeats (other.avgHOGFeatsOfSegment,feats);
  }

  float getCoplanarity(const SpectralProfile & other)
  {
    float dotproduct = getNormalDotProduct( other);
    if (fabs(dotproduct) >0.9) // if the segments are coplanar return the displacement between centroids in the direction of the normal
    {
        float distance = (centroid.x-other.centroid.x)*normal[0] + (centroid.y-other.centroid.y)*normal[1] + (centroid.z-other.centroid.z)*normal[2];
        if(distance == 0 || fabs(distance) < (1/1000) ) {return 1000;}
        return fabs(1/distance);
    }
    else  // else return -1
        return -1;
  }

  int getConvexity(const SpectralProfile & other, float mindistance)
  {
    VectorG centroid1(centroid.x,centroid.y,centroid.z);
    VectorG centroid2(other.centroid.x,other.centroid.y,other.centroid.z);
    
    VectorG c1c2=centroid2.subtract(centroid1);
    VectorG c2c1=centroid1.subtract(centroid2);
    VectorG normal1(normal[0],normal[1],normal[2]);
    VectorG normal2(other.normal[0],other.normal[1],other.normal[2]);
    if ( mindistance < 0.04 && ( (normal1.dotProduct(c1c2) <= 0 && normal2.dotProduct(c2c1) <= 0) || fabs(normal1.dotProduct(normal2)) > 0.95 ) ) // refer local convexity criterion paper
    {
        return 1;
    }
     // else return 0
    return 0;
  }

};


class BinningInfo
{
  float max;
  float min;
  int numBins;
  float binSize;
public:
  
  
  BinningInfo(float min_,float max_,int numBins_)
  {
    max=max_;
    min=min_;
    numBins=numBins_;
    assert(max>min);
    binSize=(max-min)/numBins;
    
  }

  int
  getBinIndex (float value)
  {
    assert(value>=min);
    assert(value<=max);
    
    int bin =  ((value -min) / binSize);

    assert (bin <= numBins);

    if (bin == numBins)
      {
        bin = numBins - 1;
      }
    
    return bin;

  }

  float
  GetBinSize () const
  {
    return binSize;
  }

  int
  GetNumBins () const
  {
    return numBins;
  }

  float
  GetMin () const
  {
    return min;
  }

  float
  GetMax () const
  {
    return max;
  }
};


    pcl::PointCloud<PointT> cloudUntransformed;
    pcl::PointCloud<PointT> cloudUntransformedUnfiltered;

    bool addNodeHeader=true;;
    bool addEdgeHeader=true;
    
    vector<std::string> nodeFeatNames;
    vector<std::string> edgeFeatNames;

    void addToNodeHeader(std::string featName,size_t numTimes=1)
    {
      //char featNameBuf[50];
     // assert(featName.length ()<40);
      if(addNodeHeader)
        {
          for(size_t i=0;i<numTimes;i++)
            {
       //       sprintf (featNameBuf,"%s%d",featName,i);
                nodeFeatNames.push_back (featName+boost::lexical_cast<std::string>(i));
            }
        }
    }
    
    inline void addToEdgeHeader(std::string featName,size_t numTimes=1)
    {
      //char featNameBuf[50];
      //assert(featName.length ()<40);
      if(addEdgeHeader)
        {
          for(size_t i=0;i<numTimes;i++)
            {
              //sprintf (featNameBuf,"%s%d",featName,i);
                edgeFeatNames.push_back (featName+boost::lexical_cast<std::string>(i));
            }
        }
    }
    
int MIN_SEG_SIZE=15;
#define MAX_LABEL 10
/** it also discards unlabeled segments
 */
void apply_segment_filter_and_compute_HOG(IplImage * img,Matrix<int,Dynamic,Dynamic> & segments,Matrix<int,Dynamic,Dynamic> & labels, pcl::PointCloud<PointT> &outcloud, int segment,SpectralProfile & feats) {
    //ROS_INFO("applying filter");

    outcloud.points.clear();

//    outcloud.points = incloud.points;

    int label;
    vector<int> histogram(MAX_LABEL+1,0);
    PointT point;
    for (size_t r = 0; r < img->height; r++) 
    for (size_t c = 0; c < img->width; c++) {
        label=labels(r,c);

        if (segments(r,c) ==segment && label>=0) {
            point.x=c;
            point.y=r;
            point.segment=segment;
            assert(label<=MAX_LABEL);
            histogram[label]++;
            
          outcloud.points.push_back(point);
        }
    }
    int finalLabel= std::max_element( histogram.begin(), histogram.end() ) - histogram.begin();
    if(outcloud.points.size()<MIN_SEG_SIZE)
    {
        cerr<<"segment "<<segment<<" less than "<<MIN_SEG_SIZE<<" with known labels"<<endl;
        outcloud.points.clear();
    }
    else
    {
        for(int i=0;i<outcloud.size();i++)
            outcloud.points[i].label=finalLabel;
        
        OriginalFrameInfo::findHog (  outcloud, feats.avgHOGFeatsOfSegment,originalFrame);
    }
}



pair<float,int>  getSmallestDistance (const pcl::PointCloud<PointT> &cloud1,const pcl::PointCloud<PointT> &cloud2)
{
  float min_distance = FLT_MAX;
  int min_index = 0;
  for (size_t i = 0; i < cloud1.size (); i++)
  {
  
  	
	for (size_t j = 0; j < cloud2.size (); ++j)             // nn_indices[0] should be sq_idx
      	{
			
      		float distance = pow(cloud1.points[i].x - cloud2.points[j].x,2) + pow(cloud1.points[i].y - cloud2.points[j].y,2);
      		 if (min_distance > distance)
                 {
                     min_distance = distance;
                     min_index = j;
                  //   cout << "changing min_distance to "  << min_distance<< endl;

                 }
	}
  }

  return make_pair(sqrt(min_distance),min_index) ;
}


void get_neighbors ( const std::vector<pcl::PointCloud<PointT> > &segment_clouds, map< pair <int,int> , float > &distance_matrix, map <int , vector <int> > &neighbor_map )
{
   float tolerance =30;
// get distance matrix
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
      for (size_t j = i+1; j < segment_clouds.size() ; j++)
      { 
         pair<float,int> dist_pair = getSmallestDistance(segment_clouds[i],segment_clouds[j]);
         distance_matrix[make_pair(segment_clouds[i].points[1].segment,segment_clouds[j].points[1].segment)] = dist_pair.first;
         distance_matrix[make_pair(segment_clouds[j].points[1].segment,segment_clouds[i].points[1].segment)] = dist_pair.first;
      }
 //     std::cerr<< "size of segment " << i << " : " << segment_clouds[i].points.size() << "\t and label is: " << segment_clouds[i].points[1].label <<"\n";
    }
// get neighbour map
    for ( map< pair <int,int> , float >::iterator it=distance_matrix.begin() ; it != distance_matrix.end(); it++ )
    {   
      if((*it).second < tolerance)  neighbor_map[(*it).first.first].push_back((*it).first.second);
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




void get_feature_average(vector<vector<float> > &descriptor_results, vector<float> &avg_feats) {

    vector<vector<float> >::iterator it = descriptor_results.begin();
    while (it->size() == 0) it++;
    avg_feats.resize(it->size());

    int count = 0;
    for (vector<vector<float> >::iterator it = descriptor_results.begin(); it < descriptor_results.end(); it++) {
        if (it->size() > 0) {
            count++;
        }
        vector<float>::iterator i = avg_feats.begin();
        for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++, i++) {
            *i = *i + *it2;
        }

    }
    // std::cerr << "average features" << std::endl;
    int c = 0;
    for (vector<float>::iterator i = avg_feats.begin(); i < avg_feats.end(); i++) {
        c++;
        *i = *i / count;
        //     std::cerr << c << " : " << *i << ",\t";
    }
    std::cerr << std::endl;
}

void get_feature_histogram(vector<vector<float> > &descriptor_results, vector< vector<float> > &result, vector<BinningInfo> binningInfos) {

    // num_bin = 5;
    vector<vector<float> >::iterator it = descriptor_results.begin();
    int numFeats=it->size();
    result.resize(numFeats);
    // set size of result vector
    
    vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
    for (vector<vector<float> >::iterator ires = result.begin(); ires < result.end(); ires++,binningInfo++)
      {
        ires->resize(binningInfo->GetNumBins ());       
      }
/*
    while (it->size() == 0) it++;
    max.resize(it->size(), -FLT_MAX);
    min.resize(it->size(), FLT_MAX);
    int count = 0;
    // find the bin size by finding the max and min of feature value
    for (vector<vector<float> >::iterator it = descriptor_results.begin(); it < descriptor_results.end(); it++) {
        vector<float>::iterator imax = max.begin();
        vector<float>::iterator imin = min.begin();
        int c = 0;
        for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++, imax++, imin++) {
            c++;
            //   std::cerr << c << " : " << *it2 << ",\t";
            if (*imax < *it2) {
                *imax = *it2;
            }
            if (*imin > *it2) {
                *imin = *it2;
            }
        }
        //  std::cerr << std::endl;
    }

*/
    // fill the histogram
  
    for (vector<vector<float> >::iterator it_point = descriptor_results.begin(); it_point < descriptor_results.end(); it_point++) { // iterate over points
        vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
        
        vector<vector<float> >::iterator ires = result.begin();

        assert(numFeats==it_point->size ());//missing features NOT allowed for now.
        
        for (vector<float>::iterator it_feature = it_point->begin(); it_feature < it_point->end(); it_feature++, binningInfo++, ires++) { // iterate over features of the point

            int bin = binningInfo->getBinIndex (*it_feature);
            
            //   ROS_INFO("%f %d %d",bin_size,bin,(*ires).size());
            
            (*ires)[bin] += 1;
        }
    }

    // normalize and print histogram
 //   std::cerr << "historam \n";
    
    int numPoints=descriptor_results.size ();
    
    int c1 = 0, c2 = 0;
    for (vector< vector<float> >::iterator i = result.begin(); i < result.end(); i++) {
        c1++;
   //     std::cerr << "histogram for feature:" << c1 << "\n";
        for (vector<float>::iterator i2 = i->begin(); i2 < i->end(); i2++) {
            c2++;
            *i2 = *i2 / numPoints;
            assert(*i2<=1.0);
          //  std::cerr << c2 << " : " << *i2 << ",\t";
        }
  //      std::cerr << std::endl;
    }
  //  std::cerr << std::endl;

}


// concat feats (vector-wise) to features vector

void concat_feats(vector<float> &features, vector<vector<float> > &feats) {
    for (vector<vector<float> >::iterator it = feats.begin(); it < feats.end(); it++) {

        //       vector<float>::iterator i = features.end();
        for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++) {
            features.push_back(*it2);
        }
    }
}

// concat feats vector to features vector

void concat_feats(vector<float> &features, vector<float> &feats) {
    for (vector<float>::iterator it = feats.begin(); it < feats.end(); it++) {

        features.push_back(*it);

    }
}

void get_color_features(IplImage *img,const pcl::PointCloud<PointT> &cloud, vector<float> &features, SpectralProfile & spectralProfileOfSegment) {
 int num_bin_H=9;
 int num_bin_S=4;
 int num_bin_V=4;
    // histogram and average of hue and intensity

    vector<vector<float> > hist_features;
    vector<float> avg_features;
    vector < vector <float> > color_features(cloud.points.size());
    vector <vector <float> >::iterator it = color_features.begin();
    for (size_t i = 0; i < cloud.points.size(); ++i, it++) {
        ColorRGB c=getColorOfPixel(img,cloud.points[i].y,cloud.points[i].x);
        (*it).push_back(c.H);
        (*it).push_back(c.S);
        (*it).push_back(c.V);
    }
    
    vector<BinningInfo> binnigInfos;
    binnigInfos.push_back (BinningInfo(0,360,num_bin_H));
    binnigInfos.push_back (BinningInfo(0,1,num_bin_S));
    binnigInfos.push_back (BinningInfo(0,1,num_bin_V));
    get_feature_histogram(color_features, hist_features,binnigInfos);
    get_feature_average(color_features, avg_features);
    
    spectralProfileOfSegment.avgH=avg_features[0];
    spectralProfileOfSegment.avgS=avg_features[1];
    spectralProfileOfSegment.avgV=avg_features[2];

    concat_feats(features, hist_features);addToNodeHeader ("H_hist",num_bin_H);addToNodeHeader ("S_hist",num_bin_S);addToNodeHeader ("V_hist",num_bin_V);
    
    concat_feats(features, avg_features);addToNodeHeader ("HAvg");addToNodeHeader ("SAvg");addToNodeHeader ("VAvg");
}

void get_global_features(const pcl::PointCloud<PointT> &cloud, vector<float> &features, SpectralProfile & spectralProfileOfSegment) {
    spectralProfileOfSegment.avgHOGFeatsOfSegment.pushBackAllFeats (features);addToNodeHeader ("HOG",31);
    
}

int NUM_ASSOCIATIVE_FEATS=4+31+1;
void get_pair_features( int segment_id, vector<int>  &neighbor_list,
                        map< pair <int,int> , float > &distance_matrix,
						std::map<int,int>  &segment_num_index_map,
                        vector<SpectralProfile> & spectralProfiles,
                        map < int, vector<float> > &edge_features) {

    SpectralProfile segment1Spectral=spectralProfiles[segment_num_index_map[segment_id]];

    for (vector<int>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); it++) {

        int seg2_id = *it;
        SpectralProfile segment2Spectral=spectralProfiles[segment_num_index_map[seg2_id]];
        
        
        
        
        
        //here goes the associative features:
        segment1Spectral.pushHogDiffFeats (segment2Spectral,edge_features[seg2_id]); addToEdgeHeader ("HOGDiff",31);
        
        edge_features[seg2_id].push_back(segment1Spectral.getHDiffAbs (segment2Spectral)); addToEdgeHeader ("HDiffAbs");
        
        edge_features[seg2_id].push_back(segment1Spectral.getSDiff (segment2Spectral)); addToEdgeHeader ("SDiff");
        
        edge_features[seg2_id].push_back(segment1Spectral.getVDiff (segment2Spectral)); addToEdgeHeader ("Viff");

/*        edge_features[seg2_id].push_back(segment1Spectral.getCoplanarity (segment2Spectral)); addToEdgeHeader ("Coplanarity");
        
        edge_features[seg2_id].push_back(segment1Spectral.getConvexity (segment2Spectral,distance_matrix[make_pair(segment_id,seg2_id)] ));addToEdgeHeader ("convexity");

        assert(edge_features[seg2_id].size ()==NUM_ASSOCIATIVE_FEATS);
        
        //here goes the non-associative features
        
        
        edge_features[seg2_id].push_back(segment1Spectral.getHorzDistanceBwCentroids (segment2Spectral));addToEdgeHeader ("centroid_horz_diff");
        // difference in z coordinates of the centroids
        edge_features[seg2_id].push_back(segment1Spectral.getVertDispCentroids (segment2Spectral));addToEdgeHeader ("centroid_z_diff");
        //cerr << "edge feature for edge (" << seg1_id << "," << seg2_id << ")  = " << centroid_z_diff << endl;
        
        // distance between closest points
        edge_features[seg2_id].push_back(distance_matrix[make_pair(segment_id,seg2_id)]);addToEdgeHeader ("dist_closest");

        // difference of angles with vertical
        edge_features[seg2_id].push_back(segment1Spectral.getAngleDiffInRadians (segment2Spectral));addToEdgeHeader ("AngleDiff");
		
		// dot product of normals
        edge_features[seg2_id].push_back(segment1Spectral.getNormalDotProduct (segment2Spectral));addToEdgeHeader ("NormalDotProduct");

        
        edge_features[seg2_id].push_back(segment1Spectral.getInnerness (segment2Spectral));addToEdgeHeader ("Innerness");
    
        // occupancy fraction feature
   if(UseVolFeats)
        {
                edge_features[seg2_id].push_back(get_occupancy_feature( *(segment1Spectral.cloudPtr), *(segment2Spectral.cloudPtr), tree ) );addToEdgeHeader ("Occupancy");
        }
             
// this line should be in the end
 * */
        addEdgeHeader=false;
    }
    
}
//int counts[IM_WIDTH*IM_HEIGHT];
int main(int argc, char** argv) {
  bool SHOW_CAM_POS_IN_VIEWER=false;
  if(argc!=5)
  {
      printf("usage: %s image_file segmenation_file label_file scene_num",argv[0]);
exit(-1);
  }
    int scene_num = atoi(argv[4]);
    std::ofstream nfeatfile, efeatfile;
    
    IplImage* img=0;
    img=cvLoadImage(argv[1]);
    IM_WIDTH=img->width;
    IM_HEIGHT=img->height;
    Matrix<int, Dynamic,Dynamic> segments(IM_HEIGHT,IM_WIDTH);
    Matrix<int, Dynamic,Dynamic> labels(IM_HEIGHT,IM_WIDTH);
    if(!img) printf("Could not load image file: %s\n",argv[1]);
    originalFrame=new OriginalFrameInfo(img);
    
    readCSV<int>(argv[2],IM_HEIGHT,IM_WIDTH,",",segments);
cout<<"done reading segments"<<endl;
    readCSV<int>(argv[3],IM_HEIGHT,IM_WIDTH," ",labels);
    
    nfeatfile.open("data_nodefeats.txt",ios::app);
    efeatfile.open("data_edgefeats.txt",ios::app);

    // read the pcd file



    // get segments

    // find the max segment number
    int max_segment_num = 0;

    for (int i= 0; i < IM_WIDTH*IM_HEIGHT; i++) {
//        counts[cloud.points[i].segment]++;
        if (max_segment_num < segments(i)) {
            max_segment_num = (int) segments(i);
        }
    }
    cout<< "found "<<max_segment_num<<" segments"<<endl;
    
    std::vector<pcl::PointCloud<PointT> > segment_clouds;
    std::map<int,int>  segment_num_index_map;
     pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    int index_ = 0;
    vector<SpectralProfile> spectralProfiles;
    for (int seg = 1; seg <= max_segment_num; seg++) {
        
       SpectralProfile temp;
        apply_segment_filter_and_compute_HOG (img,segments,labels,*cloud_seg,seg,temp);
        
        //if (label!=0) cout << "segment: "<< seg << " label: " << label << " size: " << cloud_seg->points.size() << endl;
        if (!cloud_seg->points.empty ()) {
         //std::cout << seg << ". Cloud size after extracting : " << cloud_seg->points.size() << std::endl;
			segment_clouds.push_back(*cloud_seg);
                        pcl::PointCloud<PointT>::Ptr tempPtr(new pcl::PointCloud<PointT > (segment_clouds[segment_clouds.size()-1]));
                        temp.cloudPtr=tempPtr;
 
                        spectralProfiles.push_back (temp);
			segment_num_index_map[seg] = index_;
			index_ ++; 
        }
    }
    map< pair <int,int> , float > distance_matrix;
    map <int , vector <int> > neighbor_map;
    cerr<<"computing neighbores"<<endl;
    clock_t start_time=clock();
    get_neighbors ( segment_clouds, distance_matrix, neighbor_map );
    clock_t elapsed=clock()-start_time;
    cerr<<"computing neighbores"<< elapsed /((double)CLOCKS_PER_SEC)<<endl;

//    cerr<<"done adding wall distance features"<<endl;

   
    // for each segment compute node featuers
    int num_bin_shape = 3;
    map < int , vector<float> > features;
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
     //   vector<float> features;
        int seg_id = segment_clouds[i].points[1].segment;
        // get color features
        //cout << "computing color features" << std::endl;
        get_color_features(img,segment_clouds[i], features[seg_id],spectralProfiles[i]);

        get_global_features(segment_clouds[i], features[seg_id],spectralProfiles[i]);// now gets HOG features
          addNodeHeader=false;

    }
    
    
    assert(nodeFeatNames.size ()<100); // some error in setting flag can cause trouble
    for(size_t i=0;i<nodeFeatNames.size ();i++)
      nfeatfile<<"#"<<nodeFeatNames[i]<<endl;
    
    for (map< int, vector<float> >::iterator it = features.begin(); it != features.end(); it++ ){
        assert(nodeFeatNames.size ()==(*it).second.size ());
        //cerr << (*it).second.size() << endl;
        nfeatfile <<  scene_num << "\t" << (*it).first << "\t" << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << "\t";
        for (vector<float>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++) {
           //cerr << *it2 << "\t";
           nfeatfile <<  *it2 << "\t";
        }
        //cerr << endl;
        nfeatfile << "\n";
    }
    cerr << "\n";
    map < int, vector <float> > edge_features;
    // find pairwise features
   // assert(edgeFeatNames.size ()<100); // some error in setting flag can cause trouble
    efeatfile<<"#"<<NUM_ASSOCIATIVE_FEATS<<endl;
    int edgecount=0;
    for ( map< int, vector<int> >::iterator it=neighbor_map.begin() ; it != neighbor_map.end(); it++) {

        edge_features.clear();
        get_pair_features((*it).first, (*it).second, distance_matrix, segment_num_index_map , spectralProfiles, edge_features);
        edgecount++;
        if(edgecount==1)
          {
                for(size_t i=0;i<edgeFeatNames.size ();i++)
                      efeatfile<<"#"<<edgeFeatNames[i]<<endl;

          }
        // print pair-wise features
        for (map< int, vector<float> >::iterator it2 = edge_features.begin(); it2 != edge_features.end(); it2++) {
          //  cerr << "edge: ("<< (*it).first << "," << (*it2).first << "):\t";
            efeatfile << scene_num << "\t" << (*it).first << "\t" << (*it2).first << "\t" << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << "\t" << segment_clouds[segment_num_index_map[(*it2).first]].points[1].label ;
                    assert(edgeFeatNames.size ()==(*it2).second.size ());

            for (vector<float>::iterator it3 = (*it2).second.begin(); it3 != (*it2).second.end(); it3++) {
            //    cerr << *it3 << "\t";
                efeatfile << "\t" <<*it3 ;
            }
            //cerr << endl;
            efeatfile << endl;
        }

    }
  
    
    cout << "DONE!!\n";
    // write features to file
/*    for (vector<float>::iterator it = features.begin(); it < features.end(); it++) {
      outfile << *it << "\t";
    }
    outfile << "\n";
 */
    //labelfile.close();
    nfeatfile.close();
    efeatfile.close();



}


