# UAV_Swarm_Control
 As the implementation is based on the [RotorS package](https://github.com/ethz-asl/rotors_simulator), first this package with all needed dependencies needs to be installed. Follow the installation instructions on their Github repository.
If you get an error if might be that you are missing the future package. Install it using the following commands: 

```
sudo apt-get install python-pip
pip install future
```

Build the cloned repository and test its functionality as described in \cite{github_rotorS} by launching a simple drone hovering example:

```
roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic`
```

Next clone the repository of this work

```
cd ~/catkin_ws/src
git clone https://github.com/laberpalaber/UAV_Swarm_Control.git
```

Finally build again the whole project

```
cd ~/catkin_ws
catkin build
```

The two controllers are listed separately, for their usage refer to the corresponding section below.

### Virtual Structure Controller
Simulations were carried out under three different conditions as described above. These simulations can be launched by their respective launch files as follows:

```
cd ~/catkin_ws/src/virtual_structure/launch
roslaunch hummingbird.launch    // for the first two simulations
roslaunch collision.launch     // for the simulation with collision
```

In the respective launch files, the initial position of the drones must be defined as required. Based on the shape and size of the formation the value of each drone's offset distance in x, y and z directions with respect to the virtual leader should be assigned to parameters *xdist, ydist* and *zdist* respectively. These parameter values need to be passed through the *fleet_control* node of the respective drone. For the first simulation condition, set the parameters *xcurrent, y current* and *zcurrent* to the same value as that of the parameters *x,y* and *z*. By setting these parameters to different values the second simulation condition can be obtained.

### Flocking Controller
Several packages were implemented for the Flocking Controller. You can launch the simulation of six hummingbird drones with the initial positions as they were used in this project by

```
cd ~/catkin_ws/src/flocking_control/launch
roslaunch swarm_hummingbird.launch
```

To change the setting simply adapt the launch file. It is important to pass the correct swarm size to the *swarm_information_pub_node* and to correctly name the drones. Additionally the correct ID needs to be passed to each drone.

To adapt the parameters for the controller or the trajectory you simply need to change them in the corresponding .yaml files. The files can be found in the *param* folder of the *swarm_controller* and *trajectory_generator* packages respectively.
