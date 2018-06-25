#include <iostream>
#include <math.h>


#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::Pose plot_position;

void plotPoseCallback(geometry_msgs::Pose Pose_received){
	plot_position = Pose_received;
	
	}

int main(int argc, char** argv) {
  ros::init(argc, argv, "adapter");
  ros::NodeHandle nh_;
 ros::Subscriber subLeaderTwist = nh_.subscribe<geometry_msgs::Pose> ("ground_truth/pose",1,plotPoseCallback);
  
 ros::Publisher plot_pose = nh_.advertise<geometry_msgs::PoseStamped>("to_plot", 1);
  ros::Rate rate(10);
  
  
  
  
  while(ros::ok())
    {
	  geometry_msgs::PoseStamped position;
      position.header.stamp = ros::Time::now();
      position.pose.position.x = plot_position.position.x;
      position.pose.position.y = plot_position.position.y;
      position.pose.position.z = plot_position.position.z;
      position.pose.orientation.x = plot_position.orientation.x;
      position.pose.orientation.y = plot_position.orientation.y;
      position.pose.orientation.z = plot_position.orientation.z;
      position.pose.orientation.w = plot_position.orientation.w;
	  
	  
      
      ros::spinOnce();
	  rate.sleep();
      plot_pose.publish(position) ;


    }

}
