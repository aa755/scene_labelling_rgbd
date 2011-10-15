#include"feat_utils.h"


    pcl::PointCloud<PointT> cloudUntransformed;
    pcl::PointCloud<PointT> cloudUntransformedUnfiltered;

int main(int argc, char** argv) {
    int scene_num = atoi(argv[2]);
    std::string unTransformedPCD=argv[3];
    std::string rgbdslamBag=argv[4];
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;
    std::ofstream labelfile, nfeatfile, efeatfile;

    //labelfile.open("data_labels.txt",ios::app);

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

    
    // convert to templated message type

    pcl::fromROSMsg(cloud_blob, cloud);

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    
          cout<<cloud.size ()<<endl;
    if (pcl::io::loadPCDFile(unTransformedPCD, cloud_blob) == -1) {
        ROS_ERROR("Couldn't read argv[3]");
        exit(-1);
    }
     
//    pcl::fromROSMsg(cloud_blob, cloudUntransformedUnfiltered);
    pcl::fromROSMsg(cloud_blob, cloudUntransformed);
    
    assert(cloudUntransformed.size()==cloud.size());
    for(size_t i=0;i<cloudUntransformed.size();i++)
        cloudUntransformed.points[i].label=cloud.points[i].label;
          pcl::PCDWriter writer;

    string base(argv[1]);
    //cout<<base.substr(12)<<endl;
     writer.write (base.substr(12),cloudUntransformed, true);


}


