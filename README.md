# Robots
Various driving and control projects

Fire-fighting robot:
- Arduino-based mobile robot featuring basic sensor integration, searching and driving. 
- Searches a grid 'city block' and scans surroundings using an IR camera. Once a source of flame has been found, system will aim its extinguisher 
(a fan in our testing) at the flame and extinguish it before returning to its original location.

Robot Arm Color Sort:
- C++ driver code with PID to operate the motors and encoders on the arm
- MATLAB scripts to search workspace for different color objects, set world coordinate waypoints, calculate kinematics and provide waypoints to 
the C++ driver for arm operation

Virtual Turtlebot SLAM in ROS:
- Simulates a Turtlebot3 in Gazebo and RViz equipped with a LIDAR for environment sensing. Procedurally explores and maps as follows:
- Robot uses available distance information to synthesize a model of the world
- Using A*, travels towards a visible point closest to predetermined goal while avoiding obstacles
- Updates map after pathfinding, travels again towards destination
- If location is equidistant from destination or enclosed by walls, A* will try alternate paths
- Once location is discovered and reached, travels back to starting location
> Have lost access to this code due to changes in Git group memberships 
