# IEEE-RAS-SIGHT-Humanitarian-Robotics-Automation-Technology-Challenge
ROS Nodes for Locomotion, Obstacle Avoidance and Mine Detection

* Unzip the "hratc2017_workspace" code folder to ~/hratc2017_workspace/src/
* The instructions below assume you have installed the HRATC 2017 code/simulator at ~/hratc2017_workspace/src
* Procedure to compile and run
  * Open move_base.launch via any text editor and please change path variable in lines (7,8,9,10,11) as per your directory
  * Open terminal and run the following commands
    * cd ~/hratc2017_workspace
    * catkin_make
  * roslaunch controller mine_detection.launch
* Components Used
  * The laser scanner, metal detectors, gps, imu is used. 
* Modules Implemented
  * A custom holonomic proportional control for p3at
  * Waypoint Generation in Lawn Mover Pattern
  * Land-Mine Detection and Avoidance
* Things to be done and tweaked in future
  * A star for Obstacle Avoidance where GPS Coordinates are given
  * Tangent Bug for new obstacles
