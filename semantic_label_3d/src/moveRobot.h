/* 
 * File:   moveRobot.h
 * Author: aa755
 *
 * Created on August 21, 2011, 3:45 PM
 */

#ifndef MOVEROBOT_H
#define	MOVEROBOT_H
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <math.h>

/**
 * not thread safe
 */
class MoveRobot
{
  ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist base_cmd;
    
    double linearConversion;
    double angularconversion;
    
    double linearSpeedMetresPerSec;
    double angularSpeedRadiansPerSeg;
    ros::NodeHandle nh_;
//    double speedDevice;
    
public:
    MoveRobot()
    {
       
    }
    
    MoveRobot(ros::NodeHandle &nh)
    {
          nh_=nh;
          cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);        
          
          linearSpeedMetresPerSec=0.1;
          angularSpeedRadiansPerSeg=0.1;
    }
    void setNodeHandle (ros::NodeHandle &nh)
    {
          nh_=nh;
          cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);        
          
          linearSpeedMetresPerSec=0.1;
          angularSpeedRadiansPerSeg=0.1;
    }
    bool moveForward(double distanceMeter, int secsToSleepAtFinish=0)
    {
        int rate=30;
        int time_to_move=(int)round(fabs(distanceMeter*rate/linearSpeedMetresPerSec));
		ros::Rate r(rate);
 		int j=0;
                setAllZero();
                
		while(nh_.ok()){
			r.sleep();
			if(j==1){
     				if(distanceMeter>=0.0)
		      			base_cmd.linear.x= linearSpeedMetresPerSec;
				else
					base_cmd.linear.x= -linearSpeedMetresPerSec;
      			}
                        
                        if(j==1+time_to_move)
      			{
                            setAllZero();
    			}
                        
                                //cout<<base_cmd.linear<<endl;
                                //cout<<base_cmd.angular<<endl;
      			cmd_vel_pub_.publish(base_cmd);
      			j++;
			if(j>2+time_to_move+rate*secsToSleepAtFinish) break;
		}
		return true;
    }
    
    //double toRadians(float)
    bool turnLeft(double angleDegree, int secsToSleepAtFinish=0)
    {
      //  DEG2RAD
        int rate=30;
        int time_to_move=(int)round(fabs(DEG2RAD(angleDegree)*rate/angularSpeedRadiansPerSeg));
		ros::Rate r(rate);
 		int j=0;
                setAllZero();
                
		while(nh_.ok()){
			r.sleep();
			if(j==1){
     				if(angleDegree>=0.0)
		      			base_cmd.angular.z= angularSpeedRadiansPerSeg;
				else
					base_cmd.angular.z= -angularSpeedRadiansPerSeg;
      			}
                        
                        if(j==1+time_to_move)
      			{
                            setAllZero();
    			}
                        
                                //cout<<base_cmd.linear<<endl;
                                //cout<<base_cmd.angular<<endl;
      			cmd_vel_pub_.publish(base_cmd);
      			j++;
			if(j>2+time_to_move+rate*secsToSleepAtFinish)break;
		}
        
        
    }
    
    void setAllZero()
    {
        base_cmd.linear.x=0;
        base_cmd.linear.y=0;
        base_cmd.linear.z=0;
        
        base_cmd.angular.x=0;
        base_cmd.angular.y=0;
        base_cmd.angular.z=0;
    }
};

#endif	/* MOVEROBOT_H */

