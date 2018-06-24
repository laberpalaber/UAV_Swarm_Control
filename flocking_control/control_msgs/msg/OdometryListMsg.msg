std_msgs/Header header

# Message that contains the odometry message of all the drones of a swarm
# swarmOdometry thus has swarmSize elements
int8 swarmSize
nav_msgs/Odometry[] swarmOdometry
