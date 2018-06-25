	#include <iostream>
	#include <fstream>
	
	#include "ros/ros.h"
	#include <geometry_msgs/Point.h>
	#include <tf/tf.h>
	#include <mav_msgs/conversions.h>
	#include <mav_msgs/default_topics.h>
	
	#include <trajectory_msgs/MultiDOFJointTrajectory.h>
	#include <Eigen/Core>
	
	
	geometry_msgs::Point leaderposture;
	bool leaderPoseReceived = false ;
	
	void leaderPoseCallback(geometry_msgs::Point leadP){
	    leaderposture = leadP ;
	    std::cout << leaderposture.x << std::endl;
	    leaderPoseReceived = true ;
	
	}
	
	int main(int argc, char** argv) {
	    ros::init(argc, argv, "control_law");
	    ros::NodeHandle nh_;
	    ros::NodeHandle nh_loc("~");
	    ros::Subscriber subLeaderTwist = nh_.subscribe<geometry_msgs::Point> ("/virtual_leader_pose",1,leaderPoseCallback);
	
	    //Publishing trajectory to the drone
	    ros::Publisher trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
	        ros::Publisher plot_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("plot", 1);
		
	
	
	
	   double yaw_val, xdist, ydist, currentx, currenty;
	    yaw_val = 1.0;
	    
	    nh_loc.param("xdist",xdist,10.0);
	    nh_loc.param("ydist",ydist,10.0);
	    
	    //to check
	    nh_loc.param("currentx",currentx,10.0);
	    nh_loc.param("currenty",currenty,10.0);
	    
	    const float DEG_2_RAD = M_PI / 180.0;
	  int i = 0;
	    ros::Rate rate(10);
	    while(ros::ok())
	    {
			//to hover
			if(i<50)
			{
				geometry_msgs::Transform transform_vals;
				trajectory_msgs::MultiDOFJointTrajectoryPoint multi_points;
			
			
				trajectory_msgs::MultiDOFJointTrajectory trajectory_message;
			
				double desired_yaw =yaw_val  * DEG_2_RAD;
				tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, desired_yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
				// Print the quaternion components (0,0,0,1)
			
				trajectory_message.header.stamp = ros::Time::now();
				trajectory_message.joint_names.push_back("joint");
			
				transform_vals.translation.x = currentx;
				transform_vals.translation.y = currenty;
				transform_vals.translation.z = leaderposture.z;
				transform_vals.rotation.x = q.getX();
				transform_vals.rotation.y = q.getY();
				transform_vals.rotation.z = q.getZ();
				transform_vals.rotation.w = q.getW();
				multi_points.transforms.push_back(transform_vals);
				trajectory_message.points.push_back(multi_points);
				trajectory_pub.publish(trajectory_message) ;
				plot_pub.publish(trajectory_message) ;

			}
			
			else
			{
						
		        geometry_msgs::Transform transform_vals;
		        trajectory_msgs::MultiDOFJointTrajectoryPoint multi_points;
		
		
		        trajectory_msgs::MultiDOFJointTrajectory trajectory_message;
		
		        double desired_yaw =yaw_val  * DEG_2_RAD;
		        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, desired_yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
		        // Print the quaternion components (0,0,0,1)
		
		        trajectory_message.header.stamp = ros::Time::now();
		        trajectory_message.joint_names.push_back("joint");
		
		        transform_vals.translation.x = leaderposture.x+xdist;
		        transform_vals.translation.y = leaderposture.y+ydist;
		        transform_vals.translation.z = leaderposture.z;
		        transform_vals.rotation.x = q.getX();
		        transform_vals.rotation.y = q.getY();
		        transform_vals.rotation.z = q.getZ();
		        transform_vals.rotation.w = q.getW();
		        multi_points.transforms.push_back(transform_vals);
		        trajectory_message.points.push_back(multi_points);
		         trajectory_pub.publish(trajectory_message) ;
				plot_pub.publish(trajectory_message) ;
		}
		i = i+1;
	
	
	
	        ros::spinOnce();
	        rate.sleep();
	
	    }
	
	
	
	
	}
	
