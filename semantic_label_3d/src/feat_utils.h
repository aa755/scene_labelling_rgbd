#include "includes/LinearLS.h"
#include "includes/matrix.h"
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

//#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
//#include <point_cloud_mapping/geometry/nearest.h>
#include <pcl_ros/io/bag_io.h>
#include "HOG.cpp"
typedef pcl::PointXYZRGBCamSL PointT;
#include "includes/CombineUtils.h"
#include<boost/numeric/ublas/matrix.hpp>
#include<boost/numeric/ublas/io.hpp>
#include<boost/numeric/bindings/traits/ublas_matrix.hpp>
#include<boost/numeric/bindings/lapack/gels.hpp>
#include <boost/numeric/bindings/traits/ublas_vector2.hpp>
namespace ublas = boost::numeric::ublas;
namespace lapack= boost::numeric::bindings::lapack;
#include "pcl_visualization/pcl_visualizer.h"
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;


//#include <Eig>
//typedef pcl::PointXYGRGBCam PointT;

typedef  pcl::KdTree<PointT> KdTree;
typedef  pcl::KdTree<PointT>::Ptr KdTreePtr;

// octomap realted

#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
using namespace octomap;

using namespace pcl;
class OriginalFrameInfo
{
  HOG hogDescriptors;
  TransformG cameraTrans;
  bool cameraTransSet;
  
public:
   pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr RGBDSlamFrame; // required to get 2D pixel positions
  
  void saveImage(int segmentId,int label,vector<Point2DAbhishek>points)
  {
  CvSize size;
  size.height=480;
  size.width=640;
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
          pcl::PointXYZRGB tmp;
  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++)
      {
        int index=x+y*size.width;
        tmp= RGBDSlamFrame->points[index];
        ColorRGB tmpColor(tmp.rgb);
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }
          
        ColorRGB tmpColor(0.0,1.0,0.0);
          for(int i=0;i<points.size ();i++)
            {
              int x=points[i].x;
              int y=points[i].y;
              
                CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
                CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
                CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
              
            }

          char filename[30];
          sprintf(filename,"s%d_l%d.png",segmentId,label);
HOG::saveFloatImage ( filename, image );
cvReleaseImage (&image);

    
  }
  void saveImage(std::string filename)
  {
  CvSize size;
  size.height=480;
  size.width=640;
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
          pcl::PointXYZRGB tmp;
  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++)
      {
        int index=x+y*size.width;
        tmp= RGBDSlamFrame->points[index];
        ColorRGB tmpColor(tmp.rgb);
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }
          

HOG::saveFloatImage ( filename.data(), image );
cvReleaseImage (&image);
  }
  
  OriginalFrameInfo(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr RGBDSlamFrame_)
  {
    cameraTransSet=false;
    RGBDSlamFrame=RGBDSlamFrame_;
  CvSize size;
  size.height=480;
  size.width=640;
  cout<<"RGBslam size:"<<RGBDSlamFrame->size ()<<endl;
    if(RGBDSlamFrame->size ()==0)
      return;// can be 0 for dummy pcds of manually transformed

  assert(RGBDSlamFrame->size ()==size.width*size.height); // can be 0 for dummy pcds of manually transformed

  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );
  
          pcl::PointXYZRGB tmp;
          
  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++)
      {
        int index=x+y*size.width;
        tmp= RGBDSlamFrame->points[index];
        ColorRGB tmpColor(tmp.rgb);
        CV_IMAGE_ELEM ( image, float, y, 3 * x ) = tmpColor.b;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = tmpColor.g;
        CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = tmpColor.r;
      }
          
          hogDescriptors.computeHog (image);
          
          cvReleaseImage (&image);
  }
  
  static Point2DAbhishek getPixelFromIndex(int index)
  {
    //assuming size is 640*480;
    int width=640;
    Point2DAbhishek ret;
    ret.y=index/width;
    ret.x=index%width;
    assert(index==ret.x+ret.y*width);
    return ret;
  }
  
  static void findHog(size_t frameIndex,vector<size_t> & pointIndices,pcl::PointCloud<PointT> &incloud, HOGFeaturesOfBlock &hogSegment, vector<OriginalFrameInfo*> & originalFrames,pcl::PointCloud<PointT> & cloudUntransformed)
  {
    static int rejectCout=0;
    OriginalFrameInfo * targetFrame=originalFrames[frameIndex];
    assert(targetFrame->RGBDSlamFrame->size ()>0);
	assert(targetFrame->cameraTransSet);
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr nnFinder(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    nnFinder->setInputCloud((targetFrame->RGBDSlamFrame));
    
    vector<int> indices;
    vector<float> distances;
    pcl::PointXYZRGB searchPoint;
    vector<Point2DAbhishek> pointsInImageLyingOnSegment;
    for(size_t i=0;i<pointIndices.size ();i++)
      {
        searchPoint.x=cloudUntransformed.points[pointIndices[i]].x;
        searchPoint.y=cloudUntransformed.points[pointIndices[i]].y;
        searchPoint.z=cloudUntransformed.points[pointIndices[i]].z;
        assert(incloud.points[pointIndices[i]].rgb==cloudUntransformed.points[pointIndices[i]].rgb); // x,y,z are transformed but color should remain same
        ColorRGB targetColor(incloud.points[pointIndices[i]].rgb);
        nnFinder->radiusSearch (searchPoint,0.0001,indices,distances,2);
        for(size_t nb=0;nb<indices.size ();nb++)
          {
            // this point could correspond to some other physical point nearby
            ColorRGB temp(targetFrame->RGBDSlamFrame->points[indices[nb]].rgb);
            if(ColorRGB::distance (temp,targetColor)<0.01)
              pointsInImageLyingOnSegment.push_back (getPixelFromIndex (indices[nb]));
            //else
              //cout<<"rejected :"<<rejectCout++<<endl;
            
          }
     
      }
    assert(pointsInImageLyingOnSegment.size ()>0);
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



class SpectralProfile
{
  vector<float> eigenValues; // sorted in ascending order
public:
  HOGFeaturesOfBlock avgHOGFeatsOfSegment;
  float avgH;
  float avgS;
  float avgV;
  pcl::PointCloud<PointT>::Ptr cloudPtr ;

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

void computeGlobalTransform(pcl::PointCloud<PointT> & combined_cloud_trans /*z aligned and possibly axis aligned*/,pcl::PointCloud<PointT> & combined_cloud_orig,TransformG & globalTrans)
{
  int numPoints=combined_cloud_orig.size ()-1;// semantics of first point not known
  //ublas::matrix<float,ublas::column_major> A(numPoints,4);
  //ublas::vector<float> b(numPoints);
  matrx <double>A(numPoints,4);
  std::vector<double> b(numPoints);


    globalTrans.transformMat(3,0)=0;
    globalTrans.transformMat(3,1)=0;
    globalTrans.transformMat(3,2)=0;
    globalTrans.transformMat(3,3)=1;
    int row;
    for(unsigned int cr=0;cr<3;cr++)
      {
        
    for(unsigned i=0;i < numPoints;i++)
        {
        
	  A.setvalue(i,0,combined_cloud_orig.points[i+1].x);
	  A.setvalue(i,1,combined_cloud_orig.points[i+1].y);
	  A.setvalue(i,2,combined_cloud_orig.points[i+1].z);
//             assert(combined_cloud_orig.points[i].x==combined_cloud_orig.points[i].data[0]);
//             assert(combined_cloud_orig.points[i].y==combined_cloud_orig.points[i].data[1]);
//             assert(combined_cloud_orig.points[i].z==combined_cloud_orig.points[i].data[2]);
	  A.setvalue(i,3,1);
             b[i]=combined_cloud_trans.points[i+1].data[cr];
        }
    //lapack::optimal_workspace works;
    //lapack::gels('N',A,b,works);
    fit(A,b,numPoints);
    
    for(unsigned int col=0;col<4;col++)
        globalTrans.transformMat(cr,col)=b[col];
    
    cout<<"row="<<cr<<endl;
    
    //check that the solution is almost correct
    for(unsigned int i=1;i < numPoints;i++)
        {
             double lhs=combined_cloud_orig.points[i].x*b[0]+combined_cloud_orig.points[i].y*b[1]+combined_cloud_orig.points[i].z*b[2]+b[3];
             double rhs=combined_cloud_trans.points[i].data[cr];
         //    cout<<lhs<<","<<rhs<<endl;
             assert(fabs(lhs-rhs)<0.01);
        }
    

      }
    globalTrans.print ();

}


void gatherOriginalFrames(std::string unTransformedPCDFile,std::string RGBDSlamBag,vector<OriginalFrameInfo*> & originalFrames,  pcl::PointCloud<PointT> & cloudUntransformed  )
{
       sensor_msgs::PointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile(unTransformedPCDFile, cloud_blob) == -1) {
        ROS_ERROR("Couldn't read argv[3]");
        exit(-1);
    }
     
//    pcl::fromROSMsg(cloud_blob, cloudUntransformedUnfiltered);
    pcl::fromROSMsg(cloud_blob, cloudUntransformed);
    /*
      pcl::PointCloud<PointT>::Ptr cloud_temp_ptr (new pcl::PointCloud<PointT > (cloudUntransformedUnfiltered));

          pcl::PassThrough<PointT> pass;
          pass.setInputCloud (cloud_temp_ptr);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (0, 10);
          pass.filter (cloudUntransformed);
          
          cout<<cloudUntransformedUnfiltered.size ()<<endl;
          cout<<"untransformed sizzes:"<<endl;
          cout<<cloudUntransformed.size ()<<endl;
    */
    
  rosbag::Bag reader;
  char *topic = "/camera/rgb/points";
  bool check;
  rosbag::View view;
  rosbag::View::iterator it;
  try
    {
      reader.open (RGBDSlamBag, rosbag::bagmode::Read);
      view.addQuery (reader, rosbag::TopicQuery ("/rgbdslam/my_clouds"));
      
      if (view.size () == 0)
	check = false;
      else
	it = view.begin ();
    }
  catch (rosbag::BagException &e)
    {
      check = false;
    }
  check = true;
  if (!check)
    {
      cout << "Couldn't open RGBDSLAM topic" << (topic);
      exit(-1);
    }
      rosbag::Bag bag;
    std::cerr << "opening " << RGBDSlamBag << std::endl;
    bag.open(RGBDSlamBag, rosbag::bagmode::Read);
  
       sensor_msgs::PointCloud2ConstPtr cloud_blob_new;
       sensor_msgs::PointCloud2ConstPtr cloud_blob_old;
       sensor_msgs::PointCloud2ConstPtr cloud_blob_temp;

       if (it != view.end ())
	 {
	   cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
	   ++it;
	 }
       cloud_blob_new = cloud_blob_temp;       
     do
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::fromROSMsg (*cloud_blob_new, *cloud_temp);
      OriginalFrameInfo * temp=new OriginalFrameInfo(cloud_temp);
        ros::Time ptime = cloud_blob_new->header.stamp;


        rosbag::View view_tf(bag, rosbag::TopicQuery("/tf"));//, ptime - ros::Duration(0, 1), ptime + ros::Duration(0, 100000000));
        //std::cerr<<(view_tf.size())<<endl;
        std::cerr << ptime << std::endl;
        //        std::cerr<<"qid:"<<pcl_ptr->header.seq<<endl;;
        int tf_count = 0;

        tf::Transform final_tft;

        BOOST_FOREACH(rosbag::MessageInstance const mtf, view_tf)
        {
            tf::tfMessageConstPtr tf_ptr = mtf.instantiate<tf::tfMessage > ();
            assert(tf_ptr != NULL);
            std::vector<geometry_msgs::TransformStamped> bt;
            tf_ptr->get_transforms_vec(bt);
            tf::Transform tft(getQuaternion(bt[0].transform.rotation), getVector3(bt[0].transform.translation));

            //  transG.print();
            if (ptime == bt[0].header.stamp)
            {
                tf_count++;
                std::cerr << "tf qid:" << bt[0].header.seq << std::endl;
                final_tft = tft;
            }
            assert(tf_count <= 1);
        }

        if(tf_count == 1)
          {
                TransformG transG(final_tft);
                transG.print ();
                temp->setCameraTrans (transG);
          }
	originalFrames.push_back (temp);
      cloud_blob_old=cloud_blob_new;
      for (int i = 0; i < 5; i++){
        if (it != view.end ())
          {
            cloud_blob_temp = it->instantiate<sensor_msgs::PointCloud2> ();
            ++it;
          }
        cloud_blob_new = cloud_blob_temp;
      }
    }
  while (cloud_blob_new != cloud_blob_old);
//    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());
  
}
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
