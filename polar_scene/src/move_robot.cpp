#include<point_types.h>
#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"
//#include "descriptors_3d/all_descriptors.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <point_cloud_mapping/geometry/nearest.h>
#include <pcl_ros/io/bag_io.h>
typedef pcl::PointXYZRGBCamSL PointT;
#include<CombineUtils.h>

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;
  double transKinBarretBase[16]={ -0.0431,-0.4261, 0.9249,-0.1156, \
-0.9874, 0.0070,-0.0455, 0.1387, \
 0.0202,-0.8612,-0.3883, 1.0495, \
0,0,0,1};
/*
typedef struct{
	float x;
	float y;
	float z;
 	int number;
  	int segment;
}centroid;
*/
      ros::Publisher pub;

class RobotDriver
{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher world_pos;
  int world_x;
  int world_y;
  int world_z;
  int flag,f;
  int time_to_move;
  double theta,fdx;
  double movex,movey,movez,center_x,center_y,center_z;
public:
  unsigned int label;		//do not change
  double threshold;	//do not change
  ros::Subscriber cloud_sub;
  ros::Subscriber get_pose;
  TransformG totalTransform;
  TransformG groundTransformInv;
  //centroid c[10];

  RobotDriver(ros::NodeHandle &nh)
  {
    label=7;
    threshold=0.0;
    flag=0;
    f=0;
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    world_pos= nh_.advertise<geometry_msgs::Twist>("/current_openrave_pos",1);
    TransformG groundTrans=readTranform("../scene_processing/globalTransform.bag");
    groundTransformInv=groundTrans.inverse();
    TransformG armTransform(transKinBarretBase);
    totalTransform=groundTransformInv.preMultiply(armTransform);
    totalTransform.print();
    
    
            
    printf("hello1\n");
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x=0;
    base_cmd.linear.y=0;
    base_cmd.angular.z=0;
    world_pos.publish(base_cmd);
  }
  
  void cameraCallback (const sensor_msgs::PointCloud2ConstPtr& point_cloud) {
      if(flag==2){
f=0;
		cout<<"hello"<<endl;
		unsigned int segment=1000000;
		int num=0;
	       	pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
	       	pcl::fromROSMsg(*point_cloud, *cloud_seg_ptr);
	       	cout<<"read a pcd of size "<<cloud_seg_ptr->size()<<endl;
		movex=0.0;
		movey=0.0;
		movez=0.0;
	       	for(unsigned int i=0;i<cloud_seg_ptr->size();i++)
	       	{
			if(cloud_seg_ptr->points[i].label==11 && segment==1000000){
				segment=cloud_seg_ptr->points[i].segment;
				f=1;
			}
			if(cloud_seg_ptr->points[i].label==11 && cloud_seg_ptr->points[i].segment==segment)
			{
				movex+=	cloud_seg_ptr->points[i].x;
				movey+= cloud_seg_ptr->points[i].y;
				movez+= cloud_seg_ptr->points[i].z;
				num++;
			}	
			//cout<<cloud_seg_ptr->points[i].x;
			//cout<<cloud_seg_ptr->points[i].label;
			//cout<<cloud_seg_ptr->points[i].segment;
	       	}
		if(f==1){
		center_x=movex/(double)num;
		center_y=movey/(double)num;
		center_z=movez/(double)num;
                PointT centroid;
                centroid.x=center_x;
                centroid.y=center_y;
                centroid.z=center_z;
                totalTransform.transformPointInPlace(centroid);
                cout<<"centroid in barret"<<endl;
                cout<<centroid.x<<","<<centroid.y<<","<<centroid.z<<endl;
                   geometry_msgs::Twist arm;
                   arm.linear.x=centroid.x;
                   arm.linear.y=centroid.y;
                   arm.linear.z=centroid.z;
                   char buf[1000];
                   sprintf(buf,"echo %f %f %f > centArm; scp centArm arm@arm-stair.cs.cornell.edu:",centroid.x,centroid.y,centroid.z);
                   system(buf);
                   //pub.publish(arm);
                centroid.x=center_x;
                centroid.y=center_y;
                centroid.z=center_z;
                groundTransformInv.transformPointInPlace(centroid);
                cout<<"centroid in kinect"<<endl;
                cout<<centroid.x<<","<<centroid.y<<","<<centroid.z<<endl;

                }
          
      }
        if(flag==0)
	{
		cout<<"hello"<<endl;
		unsigned int segment=1000000;
		int num=0;
	       	pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
	       	pcl::fromROSMsg(*point_cloud, *cloud_seg_ptr);
	       	cout<<"read a pcd of size "<<cloud_seg_ptr->size()<<endl;
		movex=0.0;
		movey=0.0;
		movez=0.0;
	       	for(unsigned int i=0;i<cloud_seg_ptr->size();i++)
	       	{
			if(cloud_seg_ptr->points[i].label==label && segment==1000000){
				segment=cloud_seg_ptr->points[i].segment;
				f=1;
			}
			if(cloud_seg_ptr->points[i].label==label && cloud_seg_ptr->points[i].segment==segment)
			{
				movex+=	cloud_seg_ptr->points[i].x;
				movey+= cloud_seg_ptr->points[i].y;
				movez+= cloud_seg_ptr->points[i].z;
				num++;
			}	
			//cout<<cloud_seg_ptr->points[i].x;
			//cout<<cloud_seg_ptr->points[i].label;
			//cout<<cloud_seg_ptr->points[i].segment;
	       	}
		if(f==1){
		center_x=movex/(double)num;
		center_y=movey/(double)num;
		center_z=movez/(double)num;
                PointT centroid;
                centroid.x=center_x;
                centroid.y=center_y;
                centroid.z=center_z;
                totalTransform.transformPointInPlace(centroid);
                cout<<"centroid for barret"<<endl;
                
		cout<<center_x<<endl;
		cout<<center_y<<endl;
		geometry_msgs::Twist base_cmd;
	    	base_cmd.linear.x=0;
    		base_cmd.linear.y=0;
    		base_cmd.angular.z=0;
    		world_pos.publish(base_cmd);	
    		//if there is a segment
		flag=1;	
		//compute theta
    		theta=atan(movey/movex)*1000.0;
		cout<<theta<<endl;
		if(theta<0.0)
			time_to_move=(int)round(((double)-theta*970.0/1570.0));
		else
			time_to_move=(int)round(((double)theta*970.0/1570.0));
		ros::Rate r(30);
 		int j=0;
		while(nh_.ok()){
			r.sleep();
			if(j>1 && j<=(1+time_to_move)){
	      			base_cmd.linear.x = 0.0;
	      			base_cmd.linear.y = 0.0;
     				if(theta>=0.0)
		      			base_cmd.angular.z= 0.1;
				else
					base_cmd.angular.z= -0.1;
      			}
      			else{
	      			base_cmd.linear.x=0.0;
      	      			base_cmd.linear.y=0.0;
              			base_cmd.angular.z=0.0;
      			}
      			cmd_vel_pub_.publish(base_cmd);
      			j++;
			if(j>2+time_to_move)break;
		}

		//compute forward motion	
		fdx=sqrt(center_x*center_x+center_y*center_y)*1000.0*0.6-200.0;
		cout<<fdx<<endl;
		time_to_move=(int)round(((double)fdx*30.0/73.629));
		j=0;
		while(nh_.ok()){
			r.sleep();
			if(j>1 && j<=(1+time_to_move)){
	      			base_cmd.linear.x = 0.1;
	      			base_cmd.linear.y = 0.0;
				base_cmd.angular.z= 0.0;
			}
      			else{
	      			base_cmd.linear.x=0.0;
      	      			base_cmd.linear.y=0.0;
              			base_cmd.angular.z=0.0;
      			}
      			cmd_vel_pub_.publish(base_cmd);
      			j++;
			if(j>2+time_to_move)break;
		}
		flag=2;
		}
		if(flag==0){
			geometry_msgs::Twist base_cmd;
			ros::Rate r(30);
			theta=-0.2*1000.0;
			cout<<"here"<<endl;
			time_to_move=(int)round(((double)-theta*970.0/1570.0));
 			int j=0;
			while(nh_.ok()){
				r.sleep();
				if(j>1 && j<=(1+time_to_move)){
	      				base_cmd.linear.x = 0.0;
		      			base_cmd.linear.y = 0.0;
     					if(theta>=0.0)
			      			base_cmd.angular.z= 0.1;
					else
						base_cmd.angular.z= -0.1;
      				}
      				else{
	      				base_cmd.linear.x=0.0;
      	      				base_cmd.linear.y=0.0;
        				base_cmd.angular.z=0.0;
      				}
      				cmd_vel_pub_.publish(base_cmd);
      				j++;
				if(j>2+time_to_move)break;
			}

		}	
	}
  }
  
  void callback(const geometry_msgs::TwistConstPtr& msg){

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver rd=RobotDriver(nh);
  //Instantiate the kinect image listener
  rd.cloud_sub=nh.subscribe("/scene_labler/labeled_cloud",1,&RobotDriver::cameraCallback,&rd);
  //rd.get_pose=nh.subscribe("/world_pose",1,&RobotDriver::callback,&rd);	
//  pub = nh.advertise<geometry_msgs::Twist>("/arm_end_effect", 1);
  ros::spin();
}

