# Running instructions

### Prequisites

Requires Ubuntu 20.04 running ROS Noetic

### Package Dependencies

*sudo apt install* 

* ros-noetic-catkin
* python3-catkin-tools
* python3-wstool
* ros-noetic-moveit
* ros-noetic-trac-ik-kinematics-plugin
* ros-noetic-moveit-kinematics
* ros-noetic-position-controllers
* ros-noetic-effort-controllers
* ros-noetic-joint-trajectory-controller
* python3-pip

*sudo pip install*

* squaternion

### Setup

* Add "export DOBOT_TYPE=nova5" to ~/.bashrc file
* Run *sudo chmod +x control_hand.py find_cube_coordinates.py* inside cube_sorting_robot/src directory to make it an executable 
