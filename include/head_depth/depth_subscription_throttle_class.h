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
#include "std_msgs/String.h"

// OpenCv
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

//image transport
#include <image_transport/image_transport.h>

class DepthSubscriptionThrottle
{
  double publ_rate_;

public:
  DepthSubscriptionThrottle(ros::NodeHandle);
  void callBackDepth(const sensor_msgs::ImageConstPtr& msg);
  void convert32FC1To8UC1(cv_bridge::CvImagePtr cv_ptr_depth, cv::Mat& depth_image_32FC1, cv::Mat& depth_image_8UC1);

protected:
  ros::NodeHandle nh_node;
  ros::Subscriber sub;

  image_transport::ImageTransport* it_;
  image_transport::Publisher depth_pub_;

  ros::Time g_last_time;
  ros::Duration g_period;

};
