/* registration main:
 * Create 
 * - a Qt Application 
 * - a ROS Node, 
 * - a ROS listener, listening to and processing kinect data
 */
#include "openni_listener.h"
//#include "openni_listener.cpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv,"hi");

  ros::NodeHandle n;
  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(n, 
//  OpenNIListener kinect_listener( 
                                 "/rgbdslam/batch_clouds",  
                                 "/rgbdslam/my_clouds" 
                                 );//,  "/camera/depth/image_raw");
 
   ros::spin();
  
//  while(1){
//   sleep(20);
   
//  }

}


