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
  unsigned int step = 10;
  
  if(argc > 1)  step = atoi(argv[2]);
  ros::NodeHandle n;
  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(n, 
//  OpenNIListener kinect_listener( 
                                 "/camera/rgb/image_mono",  
                                 "/camera/depth/image",
								 "/camera/rgb/camera_info",
								 "/camera/rgb/points",
                                 argv[1], step);//,  "/camera/depth/image_raw");
 
   ros::spin();
  
//  while(1){
//   sleep(20);
   
//  }

}


