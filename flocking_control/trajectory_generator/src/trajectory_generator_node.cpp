/**
 * \file
 * \brief
 * \author Elsa Bunz
 * \version 0.1
 * \date
 *
 * \param[in] Parameters defining the trajectory and the defined desired attitude and heading, see corresponding yaml file and main of this code
 *
 * Subscribes to: no subscriptions

 *    °
 *
 * Publishes to: trajectory_msgs/MultiDOFJointTrajectory> /UAV_swarm/trajectory
 *    °
 *
 * Node that calculates a (polynomial) trajectory of a given duration and type and publishes it.
 * Currently available types are: line, square, lemniscate and circle
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

//Necessary includes
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <eigen3/Eigen/Dense>
#include "tf/transform_datatypes.h"

// Global variables
double g_duration, g_z_des, g_z_dot_des, g_z_ddot_des, g_yaw_des, g_yaw_dot_des, g_yaw_ddot_des, g_square_size, g_lemniscate_foci_dis, g_circle_radius;
Eigen::Vector2d g_start_point, g_line_end_point;

// Method to calculate the linear trajectory
void calculateLinearTrajectory(const double t, const double duration, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, Eigen::Vector2d& pos, Eigen::Vector2d& vel, Eigen::Vector2d& acc)
{

    double a[2][6];

    for (int dim = 0; dim<2; dim++){
        // Polynomial trajectory for x and y
        a[dim][0] = start_point(dim);
        a[dim][1] = 0.0;
        a[dim][2] = 0.0;
        a[dim][3] = 10*(end_point(dim) - start_point(dim))/pow(duration,3);
        a[dim][4] = -15*(end_point(dim) - start_point(dim))/pow(duration,4);
        a[dim][5] = 6*(end_point(dim) - start_point(dim))/pow(duration,5);


        // Linear trajectory for x and y
        //        a[dim][0] = start_point(dim);
        //        a[dim][1] = (end_point(dim) - start_point(dim)) / duration;
        //        a[dim][2] = 0.0;
        //        a[dim][3] = 0.0;
        //        a[dim][4] = 0.0;
        //        a[dim][5] = 0.0;


        pos(dim) = a[dim][0] + a[dim][1]*t + a[dim][2]*pow(t,2) + a[dim][3]*pow(t,3) + a[dim][4]*pow(t,4) + a[dim][5]*pow(t,5);
        vel(dim) = a[dim][1] + 2*a[dim][2]*t + 3*a[dim][3]*pow(t,2) + 4*a[dim][4]*pow(t,3) + 5*a[dim][5]*pow(t,4);
        acc(dim) = 2*a[dim][2] + 6*a[dim][3]*t + 12*a[dim][4]*pow(t,2) + 20*a[dim][5]*pow(t,3);

    }

}

// Method to calculate the square trajectory
void calculateSquareTrajectory(const double t, Eigen::Vector2d& pos, Eigen::Vector2d& vel, Eigen::Vector2d& acc)
{
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
    double t_part;

    // Square consists of 4 lines
    if(t < 0.25 * g_duration)
    {
        start_point = g_start_point;
        end_point = g_start_point + Eigen::Vector2d (0, g_square_size);
        t_part = t;

    }else if( t < 0.5 * g_duration)
    {
        start_point = g_start_point + Eigen::Vector2d (0, g_square_size);
        end_point = g_start_point + Eigen::Vector2d (g_square_size, g_square_size);
        t_part = t - 0.25* g_duration;

    }else if (t < 0.75 * g_duration)
    {
        start_point = g_start_point + Eigen::Vector2d (g_square_size, g_square_size);
        end_point = g_start_point + Eigen::Vector2d (g_square_size, 0);
        t_part = t - 0.5* g_duration;

    }else
    {
        start_point = g_start_point + Eigen::Vector2d (g_square_size, 0);
        end_point = g_start_point;
        t_part = t - 0.75* g_duration;

    }

    calculateLinearTrajectory(t_part, 0.25*g_duration, start_point, end_point, pos, vel, acc);


}

// Method to calculate the lemniscate trajectory
void calculateLemniscateTrajectory(const double t, Eigen::Vector2d& pos, Eigen::Vector2d& vel, Eigen::Vector2d& acc)
{
    double a0,a1,a2,a3,a4,a5;

    a0 = -0.5*M_PI;
    a1 = 0.0;
    a2 = 0.0;
    a3 = 10*2*M_PI/pow(g_duration,3);
    a4 = -15*2*M_PI/pow(g_duration,4);
    a5 = 6*2*M_PI/pow(g_duration,5);

    // Polynomial trajectory
    double phi = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    double phi_dot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    double phi_ddot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);
    // Linear trajectory
    //    double phi = -0.5*M_PI + 2*M_PI/g_duration * t;
    //    double phi_dot = 2*M_PI/g_duration;
    //    double phi_ddot = 0;
    double sp = sin(phi);
    double cp = cos(phi);
    // Lemniscate parametrization: https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
    pos.x() = g_start_point.x() + (g_lemniscate_foci_dis * sqrt(2) * cp) / (sp * sp + 1);
    pos.y() = g_start_point.y() + (g_lemniscate_foci_dis * sqrt(2) * cp * sp) / (sp * sp + 1);
    vel.x() =  - g_lemniscate_foci_dis * sqrt(2)* sp * phi_dot* ((2+ cp * cp)) / pow((sp * sp +1),2);
    vel.y() = g_lemniscate_foci_dis / sqrt(2) *((2*cos(2*phi)* phi_dot)/(sp * sp +1) - phi_dot * (cp * cp - cp*cos(3*phi)) / pow((sp * sp +1),2));
    acc.x() = - g_lemniscate_foci_dis * sqrt(2)*((phi_dot * phi_dot * (2*cp + pow(cp,3) - 2* sp * sp * cp) + phi_ddot * (2*sp+sp*cp*cp))/pow((sp * sp +1),2)
                                                 - 4 * phi_dot * phi_dot* sp*sp*cp*(2+cp*cp)/pow((sp * sp +1),3));
    acc.y() = g_lemniscate_foci_dis / sqrt(2) * ((-4*sin(2*phi) * phi_dot * phi_dot + 2*cos(2*phi)* phi_ddot)/(sp*sp+1)
                                                 + (phi_dot * phi_dot*(2*sp*cp*(2*cos(2*phi) +1) - sp*cos(3*phi)- 3*cp*sin(3*phi)))/pow((sp * sp +1),2)
                                                 +(phi_ddot *(- cp*cp+cp*cos(3*phi)))/pow((sp * sp +1),2)
                                                 +(4*phi_dot *phi_dot * cp*cp*sp*(cp-cos(3*phi)))/pow((sp * sp +1),3));

    // to ensure zero values at the end
    if(t >= g_duration)
    {
        pos = g_start_point;
        vel = Eigen::Vector2d(0,0);
        acc = Eigen::Vector2d(0,0);
    }

}

// Method to calculate the circular trajectory
void calculateCircularTrajectory(const double t, Eigen::Vector2d& pos, Eigen::Vector2d& vel, Eigen::Vector2d& acc)
{
    double a0,a1,a2,a3,a4,a5;

    a0 = -M_PI;
    a1 = 0.0;
    a2 = 0.0;
    a3 = 10*2*M_PI/pow(g_duration,3);
    a4 = -15*2*M_PI/pow(g_duration,4);
    a5 = 6*2*M_PI/pow(g_duration,5);

    double phi = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    double phi_dot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    double phi_ddot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);

    pos.x() = g_start_point.x() + g_circle_radius + g_circle_radius * cos(phi);
    pos.y() = g_start_point.y() + g_circle_radius * sin(phi);
    vel.x() = - g_circle_radius * sin(phi) * phi_dot;
    vel.y() = g_circle_radius * cos(phi) * phi_dot;
    acc.x() = - g_circle_radius * (cos(phi) * phi_dot * phi_dot + sin(phi) * phi_ddot);
    acc.y() = g_circle_radius * (-sin(phi)*phi_dot*phi_dot + cos(phi) * phi_ddot);


}


// Method that calls the corresponding methods depending on the specified type and builds the message
trajectory_msgs::MultiDOFJointTrajectory calculateTrajectory(const double t, const int type)
{
    Eigen::Vector2d pos (0,0);
    Eigen::Vector2d vel (0,0);
    Eigen::Vector2d acc (0,0);


    switch(type){
    case 1:
        calculateLinearTrajectory(t, g_duration, g_start_point, g_line_end_point, pos, vel, acc);
        break;
    case 2:
        calculateSquareTrajectory(t, pos, vel, acc);
        break;
    case 3:
        calculateLemniscateTrajectory(t, pos, vel, acc);
        break;
    case 4:
        calculateCircularTrajectory(t, pos, vel, acc);
        break;
    default:
        ROS_ERROR("Trajectory type unknown (%d) - Message invalid", type);
    }

    // Create trajectory message
    trajectory_msgs::MultiDOFJointTrajectoryPoint multi_points;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_message;

    trajectory_message.header.stamp = ros::Time::now();
    trajectory_message.joint_names.push_back("joint");

    geometry_msgs::Transform transform_vals;
    transform_vals.translation.x = pos.x();
    transform_vals.translation.y = pos.y();
    transform_vals.translation.z = g_z_des;

    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, g_yaw_des);  // Create this quaternion from roll/pitch/yaw (in radians)
    transform_vals.rotation.x = q.getX();
    transform_vals.rotation.y = q.getY();
    transform_vals.rotation.z = q.getZ();
    transform_vals.rotation.w = q.getW();
    multi_points.transforms.push_back(transform_vals);

    geometry_msgs::Twist velocity_vals;
    velocity_vals.linear.x = vel.x();
    velocity_vals.linear.y = vel.y();
    velocity_vals.linear.z = g_z_dot_des;
    // !! not angular velocity, but derivative of Euler angles!!
    velocity_vals.angular.x = 0;
    velocity_vals.angular.y = 0;
    velocity_vals.angular.z = g_yaw_dot_des;
    multi_points.velocities.push_back(velocity_vals);

    geometry_msgs::Twist acceleration_vals;
    acceleration_vals.linear.x = acc.x();
    acceleration_vals.linear.y = acc.y();
    acceleration_vals.linear.z = g_z_ddot_des;
    // !! not angular velocity, but derivative of Euler angles!!
    acceleration_vals.angular.x = 0;
    acceleration_vals.angular.y = 0;
    acceleration_vals.angular.z = g_yaw_ddot_des;
    multi_points.accelerations.push_back(acceleration_vals);

    trajectory_message.points.push_back(multi_points);
    return trajectory_message;

}





int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "trajectory generator node");
    ROS_INFO("trajectory_generator_node connected.");
    ros::NodeHandle nh_glob;

    // Read the node parameters
    int trajectory_type;

    // 1: line
    // 2: square
    // 3: lemniscate
    // 4: circle
    nh_glob.param<int>("/trajectory/trajectory_type" , trajectory_type  , 1);
    nh_glob.param<double>("/trajectory/duration" , g_duration  , 10);
    nh_glob.param<double>("/trajectory/start_point/x", g_start_point.x(), 0);
    nh_glob.param<double>("/trajectory/start_point/y", g_start_point.y(), 0);
    nh_glob.param<double>("/trajectory/line/end_point/x", g_line_end_point.x(), 0);
    nh_glob.param<double>("/trajectory/line/end_point/y", g_line_end_point.y(), 0);
    nh_glob.param<double>("/trajectory/square/size", g_square_size, 0);
    nh_glob.param<double>("/trajectory/lemniscate/foci_dis", g_lemniscate_foci_dis, 0);
    nh_glob.param<double>("/trajectory/circle/radius", g_circle_radius,0);


    nh_glob.param<double> ("/flocking_control/pid_altitude/desired_altitude/z",      g_z_des,          1);
    nh_glob.param<double> ("/flocking_control/pid_altitude/desired_altitude/zd",     g_z_dot_des,          0);
    nh_glob.param<double> ("/flocking_control/pid_altitude/desired_altitude/zdd",    g_z_ddot_des,          0);
    nh_glob.param<double> ("/flocking_control/pid_heading/desired_heading/h",        g_yaw_des,          0);
    nh_glob.param<double> ("/flocking_control/pid_heading/desired_heading/hd",       g_yaw_dot_des,          0);
    nh_glob.param<double> ("/flocking_control/pid_heading/desired_heading/hdd",      g_yaw_ddot_des,          0);


    ros::Publisher pubTraj = nh_glob.advertise<trajectory_msgs::MultiDOFJointTrajectory> ("/UAV_swarm/trajectory",1);


    ros::Rate rate(30);

    // Time to hover (assumed to be done by the controller, no explicit trajectory message sent)
    ros::Duration(10.0).sleep();

    ros::Time t_start = ros::Time::now();
    double t = 0;
    while (t < g_duration){
        ros::spinOnce();

        t= (ros::Time::now() - t_start ).toSec();
        ROS_DEBUG("Call calculateTrajectory with t=%f", t);
        pubTraj.publish( calculateTrajectory(t, trajectory_type));

        rate.sleep();
    }

    pubTraj.publish( calculateTrajectory(g_duration, trajectory_type)); // ensure zero values at the end
    ROS_INFO("Trajectory finished!");

}
