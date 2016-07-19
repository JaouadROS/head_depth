/*!
*****************************************************************
*
* \note
* Project name: head detection
* \note
* ROS package name: head_depth
*
* \author
* Author: Jaouad Hajjami - jaouadhajj@gmail.com
*
* \date Date of creation: 19.07.2016
*
* \brief
* Head detection using haar detector on depth image
*****************************************************************/

// ROS
#include <ros/ros.h>

// head_detection_class
#include "head_depth/head_detection_class.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "headdetection_class");
  ros::NodeHandle nh;

  HeadDetection head_detection(nh);

  ros::spin();
  return 0;
}
