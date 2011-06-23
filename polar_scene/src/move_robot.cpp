#include"/home/nav/pkg/scene_labelling/scene_processing/src/point_types.h"
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;
/*
typedef struct{
	float x;
	float y;
	float z;
 	int number;
  	int segment;
}centroid;
*/
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
  double movex,movey,center_x,center_y;
public:
  unsigned int label;		//do not change
  double threshold;	//do not change
  ros::Subscriber cloud_sub;
  ros::Subscriber get_pose;
  
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
		cout<<"hello"<<endl;
		unsigned int segment=1000000;
		int num=0;
	       	pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
	       	pcl::fromROSMsg(*point_cloud, *cloud_seg_ptr);
	       	cout<<"read a pcd of size "<<cloud_seg_ptr->size()<<endl;
		movex=0.0;
		movey=0.0;
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
				num++;
			}	
			//cout<<cloud_seg_ptr->points[i].x;
			//cout<<cloud_seg_ptr->points[i].label;
			//cout<<cloud_seg_ptr->points[i].segment;
	       	}
		if(f==1){
		center_x=movex/(double)num;
		center_y=movey/(double)num;
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
		fdx=sqrt(center_x*center_x+center_y*center_y)*1000.0*0.6;
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
		}}
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
  rd.cloud_sub=nh.subscribe("/scene_labler/labeled_cloud",2,&RobotDriver::cameraCallback,&rd);
  //rd.get_pose=nh.subscribe("/world_pose",1,&RobotDriver::callback,&rd);	
  ros::spin();
}

