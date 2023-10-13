# Autosail_ws

This repository holds mayor part of the source code for autosail simulator

Autosail project is an investigation and collaboration project between Antioquia University, Panama Technologyc University and Malärdalen University to achieve a fully autonomous sailing boat.
This repository is part of controller branch of the project. As its main objetive is to develope a simulate platform for the project in order to protect the hardware from posible harms during 
controller testing, getting information of the device behaviour under certain conditions, and evaluating controller strategies for the autonomous sailing boat.

# Previuos to install
+ The simulator was disigned to work under ROS2: Humble so its required to have that ROS distribution on your computer you can follow the steps to a SOURCE installation from this link:https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
+ Then the simulator is also meant to function under GAZEBO:Garden simulation platform the steps to get a SOURCE install are avaliable at: https://gazebosim.org/docs/garden/install_ubuntu_src

# Install
1. With this requirements satisfied you can now install the simulator. On a new terminal or shell write the following lines
    ```
    mkdir -p ~/autosail_ws/src
    cd ~/autosail_ws/src
    git clone https://github.com/JohnkaGL/Autosail_ws.git -b main
    rosdep install -r --from-paths src -i -y --rosdistro humble
    ```
   
# After install
The bridges of the simulator need the ros_gz packages to be built this packages could be installed from cloning into the source directory of the simulator the ros_gz repository

export GZ_VERSION=garden # IMPORTANT: Replace with correct version
cd ~/autosail_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble

then you must install all of the dependencies with rosdep 

cd ~/autosail_ws
rosdep install -r --from-paths src -i -y --rosdistro humble

if it fails follow the instructions on: https://github.com/gazebosim/ros_gz/tree/humble, else continue to build the workspace:
# Source ROS distro's setup.bash
source /opt/ros/<distro>/setup.bash

# Build and install into workspace
cd ~/ws
colcon build
