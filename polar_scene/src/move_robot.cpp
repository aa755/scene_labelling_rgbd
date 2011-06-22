#include"/home/nav/pkg/scene_labelling/scene_processing/src/point_types.h"
#include <ros/ros.h>
#include <iostream>

typedef struct centroid{
	float x;
	float y;
	float z;
 	int number;
  	int segment;
}

using namespace std;

class RobotDriver
{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher world_pos;
  int world_x;
  int world_y;
  int world_z;
  int flag=0;
  const int label=0;
  const double threshold=0.0;
public:
  ros::Subscriber cloud_sub;
  ros::Subscriber get_pose;
  
  centroid c[10];

  RobotDriver(ros::NodeHandle &nh)
  {
    flag=0;
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    world_pos= nh_.advertise<geometry_msgs::Twist>("/current_openrave_pos",1);
    printf("hello1\n");
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x=0;
    base_cmd.linear.y=0;
    base_cmd.angular.z=0;
    world_pos.publish(base_cmd);
  }
  
  void cameraCallback (const sensor_msgs::PointCloud2ConstPtr& point_cloud) {
        if(flag==0)
	{
		int num=0;
	       	pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
	       	pcl::fromROSMsg(*point_cloud, *cloud_seg_ptr);
	       	cout<<"read a pcd of size "<<cloud_seg_ptr->size()<<endl;
	       	for(int i=0;i<cloud_seg_ptr->size();i++)
	       	{
			//if(cloud_seg_ptr->points[i].label==label)
				
			cout<<cloud_seg_ptr->points[i].x;
			cout<<cloud_seg_ptr->points[i].label;
			cout<<cloud_seg_ptr->points[i].segment;
	       	}
		geometry_msgs::Twist base_cmd;
	    	base_cmd.linear.x=0;
    		base_cmd.linear.y=0;
    		base_cmd.angular.z=0;
    		world_pos.publish(base_cmd);	
    		//if there is a segment
		flag=1;	
		//compute theta
    		theta=0.0;
		if(theta<0.0)
			time_to_move=(int)round(((double)-theta*970.0/1570.0));
		else
			timif(theta<0.0)
			time_to_move=(int)round(((double)-theta*970.0/1570.0));
		else
			time_to_movez=(int)round(((double)theta*970.0/1570.0));
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
		}e_to_movez=(int)round(((double)theta*970.0/1570.0));
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
		fd_x=0.0;
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
		if(flag==0){
			theta=;
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

		}	
	}
  }
  
  void callback(const geometry_msgs::TwistConstPtr& msg){

  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver rd=RobotDriver(nh);
  //Instantiate the kinect image listener
  rd.cloud_sub=nh.subscribe("/scene_labeling/labeled",2,&RobotDriver::cameraCallback,&rd);
  rd.get_pose=nh.subscribe("/world_pose",1,&RobotDriver::callback,&rd);	
  ros::spin();
}

