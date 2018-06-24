/**
 * \file
 * \brief
 * \author Elsa Bunz
 * \version 0.1
 * \date
 *
 * \param[in] droneName, swarmSize
 *
 * Subscribes to: control_msgs/OdometryListMsg /UAV_swarm/odometry
                  trajectory_msgs/MultiDOFJointTrajectory /UAV_swarm/trajectory
 *    °
 *
 * Publishes to: mav_msgs::RollPitchYawrateThrustConstPtr command/roll_pitch_yawrate_thrust
 *    °
 *
 * Controller node that calculates the desired roll-pitch-yaw angles and the desired thrust for each drone and
 * publishes it as a roll_pitch_ywarate thrust message. The implemented controll strategy is the flocking strategy described
 * in "Reactive navigation of a fleet of drones in interaction" (Osamah Saif).
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <fstream>

//ROS
#include "ros/ros.h"

// Necessary includes
#include <nav_msgs/Odometry.h>
#include <control_msgs/OdometryListMsg.h>
#include <control_msgs/OdometryNumbered.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

// To get RPY
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

// For the trajectory
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>


// define gravity
#define GRAVITY 9.81

// Define structs to store the controller parameters
struct ParameterAltitude {
    double mass;
    double Kp;
    double Kd;
    double Ki;
    std::array<double,3> alt_des;
} parameter_altitude;

struct ParameterHeading {
    double inertia_zz;
    double Kp;
    double Kd;
    double Ki;
    std::array<double,3> head_des;
} parameter_heading;

struct ParameterFlocking {
    double Kp;
    double Kd;
    double c1;
    double c2;
    double c; // Interaction distance
    double d; // desired distance
    double eps;
    double h;
    double a;
    double b;
} parameter_flocking;

// Global variables
//Publisher for the RollPitchYawrateThrust
ros::Publisher pubCommand;
//ID of the drone corresponding to the controller
int droneNum;
// For the calculation of the integrals
ros::Time t_last_att, t_last_head;
bool control_started_att = false;
bool control_started_head = false;
double integral_altitude = 0;
double integral_heading = 0;

// Initialize desired x,y and vx, vy and vx_dot, vy_dot with 0
Eigen::Vector2d qr (0,0);
Eigen::Vector2d pr (0,0);
Eigen::Vector2d pr_dot (0,0);

// data logging
bool write_data_2_file = false; // save pose data to file
std::ofstream result_file;
int swarm_size_log;

// PID controller for the altitude
double pid_altitude (nav_msgs::Odometry odom_drone)
{
    ros::Time t_current;
    ros::Duration t_delta;
    if(control_started_att){
        t_current = ros::Time::now(); // get current time
        t_delta = t_current - t_last_att; // calculate time difference for integral action
        t_last_att = t_current;
        integral_altitude += (parameter_altitude.alt_des[0] - odom_drone.pose.pose.position.z) * t_delta.toSec();
    }
    else{
        control_started_att = true;
        t_last_att = ros::Time::now();
    }

    double thrust =     parameter_altitude.alt_des[2] * parameter_altitude.mass + GRAVITY * parameter_altitude.mass
            +   parameter_altitude.Kp * (parameter_altitude.alt_des[0] - odom_drone.pose.pose.position.z)
            +   parameter_altitude.Kd * (parameter_altitude.alt_des[1] - odom_drone.twist.twist.linear.z)
            +   parameter_altitude.Ki * integral_altitude;

    ROS_DEBUG("IN PID altitude");
    return thrust;
}

// calculates the difference of two angle (angle1-angle2)
double calculateAngleDiff (double angle1, double angle2)
{
    double angle1_sub = angle1;
    double angle2_sub = angle2;

    if(angle1_sub > M_PI)
    {
        angle1_sub = angle1_sub - 2* M_PI;
    }else if(angle1_sub < - M_PI)
    {
        angle1_sub = angle1_sub + 2* M_PI;
    }

    if(angle2_sub > M_PI)
    {
        angle2_sub = angle2_sub - 2* M_PI;
    }else if(angle2_sub < - M_PI)
    {
        angle2_sub = angle2_sub + 2* M_PI;
    }

    return angle1_sub - angle2_sub;
}

// PID controller for the heading
double pid_heading (nav_msgs::Odometry odom_drone)
{
    tf::Quaternion q(
                odom_drone.pose.pose.orientation.x,
                odom_drone.pose.pose.orientation.y,
                odom_drone.pose.pose.orientation.z,
                odom_drone.pose.pose.orientation.w);
    double roll, pitch, yaw_conv;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw_conv);

    double head_des_sub = parameter_heading.head_des[0];
    double yaw_sub = yaw_conv;

    double head_dif = calculateAngleDiff(head_des_sub, yaw_sub);

    ros::Time t_current;
    ros::Duration t_delta;
    if(control_started_head){
        t_current = ros::Time::now(); // get current time
        t_delta = t_current - t_last_head; // calculate time difference for integral action
        t_last_head = t_current;
        integral_heading += head_dif * t_delta.toSec();

    }
    else{
        control_started_head = true;
        t_last_head = ros::Time::now();
    }

    // Calculate yaw_dot from omega
    // From equation 1.6 Saif Thesis
    double omega_y = odom_drone.twist.twist.angular.y;
    double omega_z = odom_drone.twist.twist.angular.z;

    double yaw_dot = (sin(roll)/cos(pitch)) * omega_y + (cos(roll)/cos(pitch)) * omega_z;

    double tau_yaw =    parameter_heading.head_des[2] * parameter_heading.inertia_zz
            +   parameter_heading.Kp * head_dif;
    +   parameter_heading.Kd * (parameter_heading.head_des[1] - yaw_dot)
            +   parameter_heading.Ki * integral_heading;

    // take the calculated yaw as rate
    double yawrate = tau_yaw;
    ROS_DEBUG("IN PID HEADING");
    return yawrate;
}



/********************************************************************************************************/
// Functions for the flocking control

// sigmaNorm for a 2d Vector
double sigmaNorm (Eigen::Vector2d z, double epsilon)
{
    double sig_norm = 1/epsilon * (sqrt(1+ epsilon * z.squaredNorm()) - 1);
    return sig_norm;

}

//sigmaNorm for a double
double sigmaNorm (double z, double epsilon)
{
    double sig_norm = 1/epsilon * (sqrt(1+ epsilon * fabs(z)*fabs(z)) - 1);
    return sig_norm;
}

Eigen::Vector2d sigmaEpsilon (Eigen::Vector2d z, double epsilon)
{
    double multiplier = 1/(1+ epsilon * sigmaNorm(z,epsilon));
    return multiplier * z;
}

double rhoH (double z, double h)
{
    double ret;
    if (0.0 <= z && z<h)
    {
        ret = 1;
    }
    else if (h <= z && z <= 1.0)
    {
        ret = 0.5 * (1+ cos(M_PI * ((z-h)/(1-h))));
    }
    else
    {
        ret = 0;
    }

    return ret;
}

double phi (double z, double a, double b)
{
    double e = fabs(a-b) / sqrt(4*a*b);
    double sigma1 = (z+e)/sqrt(1+(z+e)*(z+e));
    return 0.5 * ((a+b)*sigma1+(a-b));
}

double phiAlpha (double z, double c, double d, double epsilon, double a, double b, double h)
{
    double c_alpha = sigmaNorm(c, epsilon);
    double d_alpha = sigmaNorm(d, epsilon);
    double rho = rhoH((z/c_alpha),h);
    double p = phi((z-d_alpha),a,b);
    return rho*p;
}

// no check on whether the same drones are considered, assumption i neq j
double aij (Eigen::Vector2d qj, Eigen::Vector2d qi, double c, double epsilon, double h)
{
    double fun_arg = sigmaNorm(qj - qi, epsilon) / sigmaNorm(c, epsilon);
    return rhoH(fun_arg, h);
}

// End functions for the flocking control
/********************************************************************************************************/

// Method computing the desired roll and pitch
Eigen::Vector2d flocking_control(control_msgs::OdometryListMsg listSwarm, int droneNo){

    double eps = parameter_flocking.eps;
    double c = parameter_flocking.c;
    double d = parameter_flocking.d;
    double a = parameter_flocking.a;
    double b = parameter_flocking.b;
    double h = parameter_flocking.h;

    // get x and y position of the drone associated to the controller
    Eigen::Vector2d qi (listSwarm.swarmOdometry[droneNo-1].pose.pose.position.x, listSwarm.swarmOdometry[droneNo-1].pose.pose.position.y);
    Eigen::Vector2d pi (listSwarm.swarmOdometry[droneNo-1].twist.twist.linear.x, listSwarm.swarmOdometry[droneNo-1].twist.twist.linear.y);

    // initialize ui
    Eigen::Vector2d ui (0,0);

    //for each drone which is neighboring
    for (int drone_cnt = 0; drone_cnt < listSwarm.swarmSize; drone_cnt ++)
    {
        // we compare to ourselfs --> don't need to do that
        // +1 as numbering of the drones starts at 1
        if((drone_cnt +1) == droneNo)
        {
            continue;
        }

        // Get the position of the currently considered possible neighbor
        Eigen::Vector2d qj (listSwarm.swarmOdometry[drone_cnt].pose.pose.position.x, listSwarm.swarmOdometry[drone_cnt].pose.pose.position.y);
        // Calculate distance to the neighbor
        Eigen::Vector2d diff = qj -qi;
        double dist = diff.norm();

        ROS_DEBUG("Distance of %d to neighbor %d = %f",droneNo,drone_cnt + 1,dist);

        // Is the drone j a spatial neighbor?
        if(dist < parameter_flocking.c)
        {
            // Get velocity of the currently considered neighbor
            Eigen::Vector2d pj (listSwarm.swarmOdometry[drone_cnt].twist.twist.linear.x, listSwarm.swarmOdometry[drone_cnt].twist.twist.linear.y);

            // calculate interdistance regulation term
            Eigen::Vector2d term1 = parameter_flocking.Kp * phiAlpha(sigmaNorm((qj-qi), eps),c,d,eps,a,b,h) * sigmaEpsilon((qj-qi), eps);

            // calculate velocity consensus term
            Eigen::Vector2d term2 = parameter_flocking.Kd * aij(qj,qi,c,eps,h) * (pj - pi);

            // sum these terms over all neighboring drones
            ui += term1 + term2;

        }

    }

    // add the navigational feedback control and the acceleration
    Eigen::Vector2d term3 = - parameter_flocking.c1 * (qi-qr) - parameter_flocking.c2 * (pi - pr);
    ui += term3 + pr_dot;

    // Divide by g and -g to get roll and pitch
    Eigen::Vector2d roll_pitch(ui(1)/(-GRAVITY), ui(0)/GRAVITY);

    // return roll and pitch
    return roll_pitch;
}




// Callback for the swarm odometry information
void odometryListCallback(control_msgs::OdometryListMsg listSwarm){
    // Get the odometry message of the drone
    nav_msgs::Odometry odom_curr = listSwarm.swarmOdometry[droneNum-1];
    double roll, pitch, yawrate, thrust;

    // Call PID altitude
    thrust = pid_altitude(odom_curr);

    // Call PID heading
    yawrate = pid_heading(odom_curr);

    // call x-y flocking
    Eigen::Vector2d roll_pitch = flocking_control( listSwarm, droneNum);

    // Publish Roll-Pitch-Yawrate-Thrust message
    mav_msgs::RollPitchYawrateThrust msg;
    msg.roll = roll_pitch(0);
    msg.pitch = roll_pitch(1);
    msg.yaw_rate = yawrate;

    msg.thrust.x = 0;
    msg.thrust.y = 0;
    msg.thrust.z = thrust;

    pubCommand.publish(msg);

    // log data
    if (write_data_2_file){
        for (int i = 0; i < swarm_size_log; i++)
        {
            // Adapt the path if you want to log data
            std::string file_path = "/home/elsa/catkin_ws/src/flocking_control/log_data/position_data_drone" + std::to_string(i+1) + ".csv";
            result_file.open(file_path,std::fstream::app); // append to file
            nav_msgs::Odometry odom_curr = listSwarm.swarmOdometry[i];
            result_file.setf(std::ios::fixed, std::ios::floatfield);
            result_file.precision(5);
            tf::Quaternion q(
                        odom_curr.pose.pose.orientation.x,
                        odom_curr.pose.pose.orientation.y,
                        odom_curr.pose.pose.orientation.z,
                        odom_curr.pose.pose.orientation.w);
            double roll, pitch, yaw_conv;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw_conv);
            result_file << ros::Time::now().toSec() << ",";
            result_file << odom_curr.pose.pose.position.x << ","
                        << odom_curr.pose.pose.position.y << ","
                        << odom_curr.pose.pose.position.z << ","
                        << yaw_conv << ","
                        << qr(0) << ","
                        << qr(1) << ","
                        << parameter_altitude.alt_des[0] << ","
                        << parameter_heading.head_des[0] << ",";
            result_file << std::endl;
            result_file.close();
        }
    }

}

// Of the whole message only x,y in position, velocity and acceleration are considered!
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){

    // Retrieve the desired position and velocity
    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
    Eigen::Vector3d position = eigen_reference.position_W;
    Eigen::Vector3d velocity = eigen_reference.velocity_W;
    Eigen::Vector3d acceleration = eigen_reference.acceleration_W;


    // Save the passed desired x,y, vx,vy, vx_dot, vy_dot in the global variables
    qr(0) = position(0);
    qr(1) = position(1);

    pr(0) = velocity(0);
    pr(1) = velocity(1);

    pr_dot(0) = acceleration(0);
    pr_dot(1) = acceleration(1);

}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "Swarm_controller_node");
    ROS_INFO("Sawrm_control_node connected.");

    ros::NodeHandle nh_glob, nh_loc("~");

    // Read the node parameters
    nh_loc.param<int>    ("droneNum",                                               droneNum,                               1);
    nh_loc.param<int>    ("swarmSize",                                              swarm_size_log,                         1);


    // PID altitude
    nh_loc.param<double> ("/flocking_control/pid_altitude/mass",                    parameter_altitude.mass,            0.716);
    nh_loc.param<double> ("/flocking_control/pid_altitude/desired_altitude/z",      parameter_altitude.alt_des[0],          1);
    nh_loc.param<double> ("/flocking_control/pid_altitude/desired_altitude/zd",     parameter_altitude.alt_des[1],          0);
    nh_loc.param<double> ("/flocking_control/pid_altitude/desired_altitude/zdd",    parameter_altitude.alt_des[2],         0);
    nh_loc.param<double> ("/flocking_control/pid_altitude/gains/kp",                parameter_altitude.Kp,                  0);
    nh_loc.param<double> ("/flocking_control/pid_altitude/gains/kd",                parameter_altitude.Kd,                  0);
    nh_loc.param<double> ("/flocking_control/pid_altitude/gains/ki",                parameter_altitude.Ki,                  0);

    // PID heading
    nh_loc.param<double> ("/flocking_control/pid_heading/inertia_zz",               parameter_heading.inertia_zz,       0.012);
    nh_loc.param<double> ("/flocking_control/pid_heading/desired_heading/h",        parameter_heading.head_des[0],          0);
    nh_loc.param<double> ("/flocking_control/pid_heading/desired_heading/hd",       parameter_heading.head_des[1],          0);
    nh_loc.param<double> ("/flocking_control/pid_heading/desired_heading/hdd",      parameter_heading.head_des[2],          0);
    nh_loc.param<double> ("/flocking_control/pid_heading/gains/kp",                 parameter_heading.Kp,                   0);
    nh_loc.param<double> ("/flocking_control/pid_heading/gains/kd",                 parameter_heading.Kd,                   0);
    nh_loc.param<double> ("/flocking_control/pid_heading/gains/ki",                 parameter_heading.Ki,                   0);

    // Flocking
    nh_loc.param<double> ("/flocking_control/flocking/kp",                          parameter_flocking.Kp,                  0);
    nh_loc.param<double> ("/flocking_control/flocking/kd",                          parameter_flocking.Kd,                  0);
    nh_loc.param<double> ("/flocking_control/flocking/c1",                          parameter_flocking.c1,                  0);
    nh_loc.param<double> ("/flocking_control/flocking/c2",                          parameter_flocking.c2,                  0);
    nh_loc.param<double> ("/flocking_control/flocking/c",                           parameter_flocking.c,                   0);
    nh_loc.param<double> ("/flocking_control/flocking/d",                           parameter_flocking.d,                   0);
    nh_loc.param<double> ("/flocking_control/flocking/eps",                         parameter_flocking.eps,                 0);
    nh_loc.param<double> ("/flocking_control/flocking/h",                           parameter_flocking.h,                   0);
    nh_loc.param<double> ("/flocking_control/flocking/a",                           parameter_flocking.a,                   0);
    nh_loc.param<double> ("/flocking_control/flocking/b",                           parameter_flocking.b,                   0);

    // Subscribtion to odometry information of the swarm
    ros::Subscriber subOdomList = nh_glob.subscribe<control_msgs::OdometryListMsg> ("/UAV_swarm/odometry",1,odometryListCallback);

    // Subscription to desired trajectory
    ros::Subscriber subTraj = nh_glob.subscribe<trajectory_msgs::MultiDOFJointTrajectory> ("/UAV_swarm/trajectory",1,trajectoryCallback);

    // logging is only done in the controller node of the first drone
    if (droneNum != 1){
        write_data_2_file = false;
    }
    // enable logging
    if (write_data_2_file){
        for (int i = 0; i < swarm_size_log; i++){
            // if you want to log change the path
            std::string file_path = "/home/elsa/catkin_ws/src/flocking_control/log_data/position_data_drone" + std::to_string(i+1) + ".csv";
            //result_file = std::ofstream ();
            result_file.open(file_path);
            result_file << "time,x,y,z,yaw,x_des,y_des,z_des,yaw_des" << std::endl;
            result_file.close();
        }
    }

    // Declaration of the publisher
    pubCommand = nh_glob.advertise<mav_msgs::RollPitchYawrateThrust>(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST,1);

    ros::Rate rate(30);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

