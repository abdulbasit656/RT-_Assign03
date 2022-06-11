# Research Track 01 Assignment 03

Introduction
--------------------
The final assignment of the Research Track 1 class is based on a mobile robot simulator using the ROS framework. In this assignment I have to develop the software architecture for the control of a mobile robot in various methods. The software will rely on the ROS packges move_base and gmapping for localizing the robot and plan the autonomous motion. The software architecture must control the robot in three different ways:

    Autonomously reach a x,y coordinate inserted by the user.
    Let the user drive the robot with the keyboard.
    Let the user drive the robot with the keyboard assisting them to avoid collisions.

Installing and Running
----------------------
First Install bellow packages using following commands:
```bash
$ sudo apt-get install ros-<your_ros_distro>-navigation
$ sudo apt-get install ros-<your_ros_distro>-teleop-twist-keyboard
```
Now change directory to the source folder of your ROS workspace and git clone following packages:
```bash
$ cd ROS_ws/src
$ git clone https://github.com/CarmineD8/slam_gmapping.git
$ git clone https://github.com/abdulbasit656/RT1_Assign03.git
$ cd ..
$ catkin_make
```

Add the line ‘source [ws_path]/devel/setup.bash’ in your .bashrc file.

## Robot Simulation

To run the simulation, open a terminal and type:

    $ roslaunch rt1_assignment3 master.launch

### Simulation Launch File
The above master.launch file holds the launch parameters of both simulation_gmapping and move_base packages.
Simulation_gmapping

The simulation_gmapping package allows:

    Add the description of the robot to the ROS parameter server
    Launch the simulation in Gazebo
    Launch the Rviz node, along with some additional nodes
    Generate the robot in the simulation

Move_base
The move_base package allows:

    Launch the move_base node
    Set the rosparam described in the .yaml file


RViz & Gazebo Environment
-------------------------
![rviz](https://user-images.githubusercontent.com/17598805/172033091-767bc0f8-49b7-47c2-9517-8f57e7fcc2cf.png)
![gazebo](https://user-images.githubusercontent.com/17598805/172033095-d2150f82-1150-4ba5-b0cf-343ecba35a95.png)

## Robot Driving 
To drive the robot in simulation environment we have three modes:
  
    Autonomous drive of the robot
    Manual driving of the robot
    For manual driving of the robot with collision avoidance

To drive the robot first run the following command in new terminal:

    $ roslaunch rt1_assignment3 robot.launch

The robot.launch file allows to execute the all_modes and teleop_keyboard_twist nodes

Flowchart
---------
![RT1_assign03](https://user-images.githubusercontent.com/17598805/173172742-de52bbe6-effa-439f-b0f7-b69f1fa1a6b0.png)
---------


ROS | Python | Autonomous Drive | Teleop | Collision Avoidance 
