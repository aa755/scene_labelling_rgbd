#include "feat_utils.h"
#include "segmentAndLabel.h"

using namespace pcl;


void writeToBag(std::string outFile,pcl::PointCloud<PointT> frame_cloud,TransformG frameTrans)
{
            rosbag::Bag bag_;
              bag_.open(outFile, rosbag::bagmode::Write);

        sensor_msgs::PointCloud2Ptr cloud_out_blob(new sensor_msgs::PointCloud2());

              pcl::toROSMsg (frame_cloud, *cloud_out_blob);
          ros::Time ftime=ros::Time::now ();
          cloud_out_blob->header.stamp=ftime;
          cloud_out_blob->header.seq=0;
          
          bag_.write ("/rgbdslam/my_clouds", ftime, cloud_out_blob);
          geometry_msgs::TransformStamped gmMsg;
          tf::transformStampedTFToMsg (tf::StampedTransform(frameTrans.getAsRosMsg (), ftime,"/openni_camera", "/batch_transform"),gmMsg);
          tf::tfMessage tfMsg;
          tfMsg.set_transforms_size (1);
                      std::vector<geometry_msgs::TransformStamped> bt;
                      bt.push_back (gmMsg);

          tfMsg.set_transforms_vec (bt);
          bag_.write("/tf",ftime,tfMsg);
         // tf::tfMessage
          bag_.close();
  
}


int main(int argc, char** argv) {
pcl::PointCloud<PointT> cloudUntransformed;
  ros::Time::init();

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

    for(unsigned int i=0;i<originalFrames.size ();i++)
      originalFrames[i]->applyPostGlobalTrans (globalTransform);
    
    pcl::PointCloud<PointT> temp1;
    pcl::PointCloud<PointT> temp2;
    
    vector<pcl::PointCloud<PointT>::Ptr> outClouds;
    vector<TransformG> outTrans;
          string base(argv[1]);
          bool first=true;
          TransformG prevCam;
    for(size_t i=0;i<originalFrames.size ();i++)
      {
        if(!originalFrames[i]->isEmpty ())
          {
            if(!first&&prevCam.isOverlapSignificant(originalFrames[i]->getCameraTrans()))
            {
//                cerr<<"overlap rejected";
                continue;
            }
            first=false;
            
            originalFrames[i]->saveImage(boost::lexical_cast<std::string>(i)+"_"+base+".png");
                pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT> ());
                findlabel (cloudUntransformed,*originalFrames[i]->RGBDSlamFrame,temp1,originalFrames[i]->getCameraTrans ().getOrigin (),i);
                
                segment (temp1, temp2);
                cerr<<"ambiguities of "<<argv[1]<<outClouds.size()<<endl;
                findConsistentLabels (temp2,*cloud_temp);
                globalTransform.transformPointCloudInPlaceAndSetOrigin (*cloud_temp);
                outClouds.push_back (cloud_temp);
                outTrans.push_back(originalFrames[i]->getCameraTrans());
                prevCam=originalFrames[i]->getCameraTrans();
          }
      }
   //   pcl::PCDWriter writer;

    for(size_t i=0;i<outClouds.size ();i++)
      writeToBag ( boost::lexical_cast<std::string>(i)+"_"+base+".bag",*outClouds[i],outTrans[i]);
   //   writer.write ( boost::lexical_cast<std::string>(i)+"_"+base,*outClouds[i], true);
      
    
}


