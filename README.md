# head_depth
Head detection using haar detector on depth image

This package has been tested using:

*  ROS Indigo
*  Ubuntu 14.04
*  Kinect v1

The purpose of this package is to detect head in depth image using haar detector

Quick Start:
===============

To use this package, clone the repository to your disc and build it:

    $ cd catkin_ws/src 
    $ git clone https://github.com/JaouadROS/head_depth
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash

Run head_depth launch file

    $ roslaunch head_depth head_depth.launch

```sh
SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20

NODES
  /
     depth_subscription_throttle (head_depth/depth_subscription_throttle)
     head_detection (head_depth/head_detection)
```

The head detected is displayed in a depth image as a white rectangle. 

The first node depth_subscription_throttle is publishing depth image in 8UC1 format in 15 fps. After that the node head_detection is subscribing to it and detecting heads (or anything similar to it, like a Vase with flowers! ) using haar detector.
