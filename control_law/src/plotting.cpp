#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
//Variables: actual
nav_msgs::Odometry d1_actual;
nav_msgs::Odometry d2_actual;
nav_msgs::Odometry d3_actual;
nav_msgs::Odometry d4_actual;

//Variables desired
trajectory_msgs::MultiDOFJointTrajectory des_1;
trajectory_msgs::MultiDOFJointTrajectory des_2;
trajectory_msgs::MultiDOFJointTrajectory des_3;
trajectory_msgs::MultiDOFJointTrajectory des_4;

bool d1Received = false;
bool d2Received = false;
bool d3Received = false;
bool d4Received = false;

bool des1Received = false;
bool des2Received = false;
bool des3Received = false;
bool des4Received = false;



//callback functions for actual values

void drone1Callback(nav_msgs::Odometry d1_call){
	d1_actual = d1_call;
	d1Received = true;
}

void drone2Callback(nav_msgs::Odometry d2_call){
	d2_actual = d2_call;
	d2Received = true;
}

void drone3Callback(nav_msgs::Odometry d3_call){
	d3_actual = d3_call;
	d3Received = true;
}

void drone4Callback(nav_msgs::Odometry d4_call){
	d4_actual = d4_call;
	d1Received = true;
} 

//Callback functions for desired trajectory

void desired1Callback(trajectory_msgs::MultiDOFJointTrajectory des_1_call){
des_1 = des_1_call;
des1Received = true;
} 

void desired2Callback(trajectory_msgs::MultiDOFJointTrajectory des_2_call){
des_2 = des_2_call;
des2Received = true;
} 

void desired3Callback(trajectory_msgs::MultiDOFJointTrajectory des_3_call){
des_3 = des_3_call;
des3Received = true;
} 

void desired4Callback(trajectory_msgs::MultiDOFJointTrajectory des_4_call){
des_4 = des_4_call;
des4Received = true;
} 
int main(int argc, char** argv) {
    ros::init(argc, argv, "plotting");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_loc("~");
    
    //Subscribers for actual values
    ros::Subscriber drone1 = nh_.subscribe<nav_msgs::Odometry> ("/hummingbird1/ground_truth/odometry",1,drone1Callback);
	ros::Subscriber drone2 = nh_.subscribe<nav_msgs::Odometry> ("/hummingbird2/ground_truth/odometry",1,drone2Callback);
	ros::Subscriber drone3 = nh_.subscribe<nav_msgs::Odometry> ("/hummingbird3/ground_truth/odometry",1,drone3Callback);
	ros::Subscriber drone4 = nh_.subscribe<nav_msgs::Odometry> ("/hummingbird4/ground_truth/odometry",1,drone4Callback);
		
	//Subscribers for desired values	
	ros::Subscriber desired1 = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory> ("/hummingbird1/plot",1,desired1Callback);
	ros::Subscriber desired2 = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory> ("/hummingbird2/plot",1,desired2Callback);
	ros::Subscriber desired3 = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory> ("/hummingbird3/plot",1,desired3Callback);
	ros::Subscriber desired4 = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory> ("/hummingbird4/plot",1,desired4Callback);

    ros::Rate rate(10);
    while(ros::ok())
    {
	if(des1Received == true && des2Received == true && des3Received == true && des4Received == true && d1Received == true && d2Received == true && d3Received == true && d4Received == true)
	{
	
		
	}
 

        ros::spinOnce();
        rate.sleep();

    }




}
