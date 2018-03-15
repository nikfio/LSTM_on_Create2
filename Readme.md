# LSTM as global path planner on a real mobile robot

This is an attempt to develop a global path planner:
	- high level
	- low resolution
using the Long Short-Term Memory neural network, for a real robot implementation.

This project is made for the IRobot Create 2 and Hokuyo UTM-30LX-EW (but any of Hokuyo UTM series should work).
	
In particular, this is not an offline search like A* or similar, but it is an online search agent.
The main feature and advantage is memory usage: just the current state occupies the agent memory, while A* has to potentially store the all map. 
In a real robot, memory usage for an online agent is a very tiny little fraction compared to offline agents.
But, for me implementation of this online agent was complicated, this repo can build an agent for very simple - meaning quite close - goal points.

Developing and testing on Ubuntu 14.04 LTS Trusty.
Core libraries: cuda-8.0 working with GTX 1080Ti, CAFFE latest from the master branch of the main repository, ROS-jade.

Build instructions:

    cmake files search for caffe in /usr/local, just like for cuda, fastest way is to copy your caffe build in there

    place terminal in home directory

    rm -r build (I usually upload already built versions, so you have to delete everthing of the existing build)

    catkin_make



If you have suggestions or comments, please feel free to contact me.
