# openai_ros

## Capabilities
 
This **ROS Noetic** package provides similiar functions to the [panda-gym](https://github.com/qgallouedec/panda-gym) package, but based on Gazebo. Here, we implemented several robotics environments to allow using this package to perform Reinforcement Learning on tasks, specifically for the **Franka Emika Robot**. It is directly modified from , and follows similar structure with the [openai_ros](https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/) package by TheConstructCore.

## Installation

Execute the following commands:<br>
`cd ~/ros_ws/src`<br>
`git clone https://github.com/angiecodinghub/openai_ros.git src/openai_ros`<br>
`cd ~/ros_ws`<br>
`catkin_make`<br>
`source devel/setup.bash`<br>
`rosdep install openai_ros`<br>

## Available Task Environments

1. Reach: <br>
The goal is to let the end effector of the robot reach a certain location in the space. 
* gripper: blocked. 
* maximum step per episode: 50.
* init position: 90 degrees at its elbow: <br>
            {'panda_joint1': 0.0, <br>
            'panda_joint2': 0.0, <br>
            'panda_joint3': 0.0, <br>
            'panda_joint4': -1.57079632679, <br>
            'panda_joint5': 0.0, <br>
            'panda_joint6': 1.57079632679, <br>
            'panda_joint7': 0.785398163397]
* control types: <br>
"ee": learn the displacement of the end effector. <br>
"joint": learn the displacement (of angles) of the 7 joints.

## Contact Info

Maintainer: Angela Wu (annwu@rice.edu) <br>
Reachable via Slack as well.





