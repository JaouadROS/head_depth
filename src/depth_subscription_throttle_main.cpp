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

//Camera_Height header
#include "head_depth/depth_subscription_throttle_class.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depthsubscription_throttle");
  ros::NodeHandle nh;

  DepthSubscriptionThrottle depth_sub(nh);

  ros::spin();
  return 0;
}
