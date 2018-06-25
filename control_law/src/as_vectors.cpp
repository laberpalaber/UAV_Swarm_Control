#include <iostream>
#include<vector>


#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_law");
  ros::NodeHandle nh_;
//  ros::Publisher pubLeaderPosture
  //       = nh_.advertise<geometry_msgs::Vector3>("virtual_leader_pose",1);

std_msgs::Float64MultiArray x_vals;
std_msgs::Float64MultiArray y_vals;
std_msgs::Float64MultiArray z_vals;



ros::Publisher pubLeaderPosturex
         = nh_.advertise<std_msgs::Float64MultiArray>("virtual_leader_pose_x",1);

ros::Publisher pubLeaderPosturey
         = nh_.advertise<std_msgs::Float64MultiArray>("virtual_leader_pose_y",1);


ros::Publisher pubLeaderPosturez
         = nh_.advertise<std_msgs::Float64MultiArray>("virtual_leader_pose_z",1);


 // std::vector<float> x_vals;
 // std::vector<float> y_vals;
 // std::vector<float> z_vals;
  
  
  ros::Rate rate(1);
  float m ,x_int, y_int, z_int ;
  m = 0.0;
  x_int = 1.0; 
  y_int = 1.0;
  z_int = 0.0;
  
  
x_vals.layout.dim.push_back(std_msgs::MultiArrayDimension());
x_vals.layout.dim[0].size = 20;
x_vals.layout.dim[0].stride = 1;
x_vals.layout.dim[0].label = "xvalues";
 
y_vals.layout.dim.push_back(std_msgs::MultiArrayDimension());
y_vals.layout.dim[0].size = 20;
y_vals.layout.dim[0].stride = 1;
y_vals.layout.dim[0].label = "yvalues";

z_vals.layout.dim.push_back(std_msgs::MultiArrayDimension());
z_vals.layout.dim[0].size = 20;
z_vals.layout.dim[0].stride = 1;
z_vals.layout.dim[0].label = "zvalues"; 
 
 
  while(ros::ok() && m<20)
    {
		
		x_vals.data.push_back(x_int*m);
		y_vals.data.push_back(y_int*m);
		z_vals.data.push_back(z_int*m);
    m = m+1;
    }
     pubLeaderPosturex.publish(x_vals) ;
      pubLeaderPosturey.publish(y_vals) ;
      pubLeaderPosturez.publish(z_vals) ;
     ros::spinOnce();
	  rate.sleep();
     

}
