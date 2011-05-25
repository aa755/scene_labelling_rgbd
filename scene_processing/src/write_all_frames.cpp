#include "feat_utils.h"
#include "segmentAndLabel.h"

using namespace pcl;




int main(int argc, char** argv) {
pcl::PointCloud<PointT> cloudUntransformed;

vector<OriginalFrameInfo*> originalFrames;
    int scene_num = atoi(argv[2]);
    std::string unTransformedPCD=argv[3];
    std::string rgbdslamBag=argv[4];
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;

    //labelfile.open("data_labels.txt",ios::app);

    // read the pcd file

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

    
    // convert to templated message type

    pcl::fromROSMsg(cloud_blob, cloud);

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    
          cout<<cloud.size ()<<endl;
    gatherOriginalFrames (unTransformedPCD,rgbdslamBag,originalFrames,cloudUntransformed);
    TransformG globalTransform;
    computeGlobalTransform (cloud,cloudUntransformed,globalTransform);

   // for(unsigned int i=0;i<originalFrames.size ();i++)
   //   originalFrames[i]->applyPostGlobalTrans (globalTransform);
    pcl::PointCloud<PointT> temp1;
    pcl::PointCloud<PointT> temp2;
    
    vector<pcl::PointCloud<PointT>::Ptr> outClouds;
    for(size_t i=0;i<originalFrames.size ();i++)
      {
        if(!originalFrames[i]->isEmpty ())
          {
                pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT> ());
                findlabel (cloudUntransformed,*originalFrames[i]->RGBDSlamFrame,temp1,originalFrames[i]->getCameraTrans ().getOrigin (),i);
                segment (temp1, temp2);
                findConsistentLabels (temp2,*cloud_temp);
                globalTransform.transformPointCloudInPlaceAndSetOrigin (*cloud_temp);
                outClouds.push_back (cloud_temp);
          }
      }
      pcl::PCDWriter writer;

      string base(argv[1]);
    for(size_t i=0;i<outClouds.size ();i++)
      writer.write ( base+boost::lexical_cast<std::string>(i),*outClouds[i], true);
      
    
}


