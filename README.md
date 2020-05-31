# Clifford Robot Simulation
The ROS package includes a Gazebo simulation of the clifford robot. Robot can be controlled through publishing commands a twist message to the ros topic "/cliffordDrive." Currently, twist's linear x controls the throttle of the robot and takes values between -1 and 1. The twist's angular z controls the forward steering and takes values between -1 and 1

## Basic Example
As a basic example, run the launch file "clifford_empty_world.launch". This will start a Gazebo simulation with an empty world and import the clifford model. The user can then run the script "mouseControl.py" which will launch a GUI to control Clifford. If the user clicks and drags in the GUI, the program will convert the drag direction into a twist and command it to "/cliffordDrive".

## Random Terrain Example
As another example, run the launch file "fast_empty_world.launch". this will start a empty Gazebo simulation that runs with the fastest possible real time factor. The user can then run the script "randomTerrainTrajFollow.py" This script imports a random terrain and the clifford model into the Gazebo simulation. It then chooses a random trajectory to follow and uses a PD controller to track the trajectory.
