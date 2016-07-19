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
#include "ros/ros.h"
#include <ros/package.h>

// OpenCv
#include <opencv/cv.h>

// vector
#include <vector>

// image transport
#include <image_transport/image_transport.h>

class HeadDetection
{
  std::string haar_cascade_file;
  std::vector<cv::Rect> rect_heads;
  cv::CascadeClassifier classifier_cascade;

public:
  HeadDetection(ros::NodeHandle);
  void callBackDepth(const sensor_msgs::ImageConstPtr& msg);

protected:
  ros::NodeHandle nh_node;
  ros::Subscriber depth_sub_;

  ros::Time g_last_time;
  ros::Duration g_period;
};

















// %EndTag(CLASS_WITH_DECLARATION)%
