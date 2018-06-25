#include <iostream>
#include <math.h>


#include "ros/ros.h"
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_law");
  ros::NodeHandle nh_;
  ros::Publisher pubLeaderPosture
         = nh_.advertise<geometry_msgs::Point>("/virtual_leader_pose",1);
  
  ros::Rate rate(10);
  float m ,x_int, y_int, z_int ;
  m = 0.0;
  x_int = 1.0;
  y_int = 1.0;
  z_int = 1.0;
  //Values for spiral trajectory
  float d, r, theta;
  d = 0.0;
  r = 0.0;
  theta = 0;
  geometry_msgs::Point position;
  while(ros::ok())
    {
		
	//x=y=z trajectory	
		/*
      position.x = x_int*m;
      position.y = y_int*m;
      position.z = z_int*m;
      m = m+0.02;
      */
      //End of x=y=z Trajectory

      
      //Spiral Trajectory
      position.x = d*cos(r);
      position.y = d*sin(r);
      position.z = d;
      d = d+0.01;
      r = r+2*M_PI/99;
      //End of Spiral Trajectory
      /*
      //Circular Trajectory
      //radius = 5;
      position.x = 5.0*cos(theta);
      position.y = 5.0*sin(theta);
      position.z = 1.5;
      theta = theta + 2*M_PI/99; 
      */
      ros::spinOnce();
	  rate.sleep();
      pubLeaderPosture.publish(position) ;


    }

}
