#ifndef SWARM_CONTROLLER_NODE
#define SWARM_CONTROLLER_NODE

#endif // SWARM_CONTROLLER_NODE

#pragma once

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
#include <nav_msgs/Odometry.h>
#include <control_msgs/OdometryListMsg.h>
#include <control_msgs/OdometryNumbered.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
