/**
 * \file
 * \brief
 * \author Elsa Bunz
 * \version 0.1
 * \date
 *
 * \param[in] droneName, swarmSize
 *
 * Subscribes to: nav_msgs/Odometry /droneNamei/ground_truth/odometry
 *    °
 *
 * Publishes to: control_msgs/OdometryListMsg UAV_swarm_informations/odometry
 *    °
 *
 * Node subscribes to the odometry messages of the drones present in the swarm and publishes them in one message
 * Assumes drones are numbered from 1 until swarmSize
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
#include <nav_msgs/Odometry.h>
#include <control_msgs/OdometryListMsg.h>
#include <control_msgs/OdometryNumbered.h>

//Global variables
ros::Publisher odomListPub;
// no vector of bool because of http://www.codingstandard.com/rule/17-1-1-do-not-use-stdvector/
std::vector<int32_t> odomReceived;
std::vector<nav_msgs::Odometry> odomList;

// Odometry callback
void odometryMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry, int droneNo){

    ROS_DEBUG("Was in odometry callback");

    // Already received an odometry message of this drone
    // should not happen
    // -1 because drone numbering starts at 1
    if (odomReceived[droneNo-1]== 1)
    {
        ROS_WARN("Odometry message overwritten");
    }
    else
    {
        odomReceived[droneNo-1] = 1;
    }
    odomList[droneNo-1] = *odometry;

    // check whether odometry message of all drones was received
    bool allOdomReceived = true;
    for(int i=0; i< odomReceived.size(); i++)
    {
        if(odomReceived[i] == 0)
        {
            allOdomReceived = false;
        }
    }


    // Callback was called by all drones in the swarm
    if(allOdomReceived)
    {
        // Reset odomReceived
        for(int i=0; i< odomReceived.size(); i++)
        {
            odomReceived[i] = 0;
        }

        // Build the message
        control_msgs::OdometryListMsg listSwarm;
        listSwarm.swarmSize = odomReceived.size();
        listSwarm.swarmOdometry = odomList;

        // Publish the message
        odomListPub.publish(listSwarm);
    }
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "Swarm info pub node");
    ROS_INFO("Swarm info pub node connected.");

    // Define your node handles
    ros::NodeHandle nh_glob, nh_loc("~");

    // Read the node parameters
    std::string droneName;
    int swarmSize;

    nh_loc.param<std::string>("droneName" , droneName  , "hummingbird");
    nh_loc.param("swarmSize"   , swarmSize    , 2);

    // name of subscribed topic
    std::string topicName = "/ground_truth/odometry";

    // to initialize odomList
    nav_msgs::Odometry odom ;

    // to store the subscribers
    std::vector<ros::Subscriber> subscriberList;

    // Declaration of the subscriptions and initialization of global variables
    for(int drone_cnt=1; drone_cnt<= swarmSize; drone_cnt++)
    {
        std::string fullTopicName = "/" + droneName + std::to_string(drone_cnt) + topicName;
        ROS_INFO("Subscribed topic: %s", fullTopicName.c_str());

        ros::Subscriber sub = nh_glob.subscribe<nav_msgs::Odometry> (fullTopicName,1,boost::bind(odometryMsgCallback, _1, drone_cnt));
        subscriberList.push_back(sub);

        odomReceived.push_back(0);
        odomList.push_back(odom);
    }

    // Declaration of publisher
    odomListPub = nh_glob.advertise<control_msgs::OdometryListMsg>("/UAV_swarm/odometry",1);

    ros::Rate rate(30);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
