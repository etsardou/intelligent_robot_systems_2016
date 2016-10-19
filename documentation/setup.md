##Installation - Setup

This series of challenges require the utilization of the **STDR Simulator** which is a collection **ROS** packages. The operating system required is **Ubuntu 14.04**.

[ROS](http://www.ros.org/) is a "flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms." 

[STDR Simulator](http://stdr-simulator-ros-pkg.github.io/) is a two dimentional robot simulator that enables simple simulation of robots in an intuitive and easy way. It is based on ROS and its main goal is to enable you to setup a simulation environment in just a few minutes.

###ROS installation

First of all you must install ROS. The flavour is ROS Indigo if you have Ubuntu 14.04 installed, or ROS hydro for Ubuntu 12.04. The installation instructions can be found here:
- [ROS Indigo installation](http://wiki.ros.org/indigo/Installation/Ubuntu)

###Download the required packages

The required packages you have to download are STDR Simulator and the current repository. Before them install the following libraries:

```
sudo apt-get install git mercurial ros-indigo-map-server
```

Create a catkin repository in a folder you want (in this tutorial we create in in $HOME):
```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Next, clone the STDR Simulator:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git
cd stdr_simulator
git checkout autonomous_systems
```

Next, clone the Intelligent Robotic System's repository:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/etsardou/intelligent_robot_systems_2016.git
```

Build the packages:
```bash
cd ~/catkin_ws
catkin_make

cd ~/catkin_ws/src/intelligent_robot_systems_2016/art_autonomous_exploration/src
make
```
If the build was successful you are ok!

