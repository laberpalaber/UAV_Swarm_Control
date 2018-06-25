#include <iostream>
#include <fstream>

#include "ros/ros.h"

#include <tf/tf.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <std_msgs/Float64MultiArray.h>

//Declaring objects to copy in callback and use
std_msgs::Float64MultiArray x_v;
std_msgs::Float64MultiArray y_v;
std_msgs::Float64MultiArray z_v;
std_msgs::Float64MultiArray yaw_vals;

bool xReceived = false;
bool yReceived = false;
bool zReceived = false;


//Callbacks for the trajectory values
void leaderPoseXCallback(std_msgs::Float64MultiArray leadPx){
    x_v = leadPx ;
	xReceived = true;


}
void leaderPoseCallback(std_msgs::Float64MultiArray leadPy){
    y_v = leadPy ;
	yReceived = true;


}
void leaderPoseCallback(std_msgs::Float64MultiArray leadPz){
    z_v = leadPz ;
	zReceived = true;


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_law");
    ros::NodeHandle nh_;
    ros::Subscriber subLeaderX = nh_.subscribe<std_msgs::Float64MultiArray> ("virtual_leader_pose_x",1,leaderPoseXCallback);
    ros::Subscriber subLeaderY = nh_.subscribe<std_msgs::Float64MultiArray> ("virtual_leader_pose_y",1,leaderPoseYCallback);
    ros::Subscriber subLeaderZ = nh_.subscribe<std_msgs::Float64MultiArray> ("virtual_leader_pose_z",1,leaderPoseZCallback);

    //Publishing trajectory to the drone
    ros::Publisher trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("mav_msgs::default_topics::COMMAND_TRAJECTORY", 1);
    
    ROS_INFO("Started as_vector_receiver.");
	double delay;
	delay = 2;
   double yaw_val, xdist, ydist;
    yaw_val = 1.0;
    
    nh_.param("/xdist",xdist,10.0);
    nh_.param("/ydist",ydist,10.0);
    
    //creating yaw_vals vector
    yaw_vals.layout.dim.push_back(std_msgs::MultiArrayDimension());
	yaw_vals.layout.dim[0].size = 20;
	yaw_vals.layout.dim[0].stride = 1;
	yaw_vals.layout.dim[0].label = "xvalues";
	for(int i = 0; i<20;i++)
	{
		yaw_vals.data.push_back(0);
	}
    
    const float DEG_2_RAD = M_PI / 180.0;

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();

  Eigen::Vector3d desired_position(x_v.data, y_v.data,
                                   z_v.data);

  double desired_yaw = yaw_vals.data * DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;














    ros::Rate rate(1);
    while(ros::ok())
    {

        geometry_msgs::Transform transform_vals;
        trajectory_msgs::MultiDOFJointTrajectoryPoint multi_points;


        trajectory_msgs::MultiDOFJointTrajectory trajectory_message;

        double desired_yaw =yaw_val  * DEG_2_RAD;
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, desired_yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
        // Print the quaternion components (0,0,0,1)

        trajectory_message.header.stamp = ros::Time::now();
        trajectory_message.joint_names.push_back("joint");

        transform_vals.translation.x = leaderposture.x;
        transform_vals.translation.y = leaderposture.y;
        transform_vals.translation.z = leaderposture.z;
        transform_vals.rotation.x = q.getX();
        transform_vals.rotation.y = q.getY();
        transform_vals.rotation.z = q.getZ();
        transform_vals.rotation.w = q.getW();
        multi_points.transforms.push_back(transform_vals);
        trajectory_message.points.push_back(multi_points);


        trajectory_pub.publish(trajectory_message) ;

        ros::spinOnce();
        rate.sleep();

    }




}
