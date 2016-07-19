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

// OpenCv
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// depth subscription throttle
#include "head_depth/depth_subscription_throttle_class.h"

DepthSubscriptionThrottle::DepthSubscriptionThrottle(ros::NodeHandle nh) :	nh_node(nh)
{
  publ_rate_=20;
  g_period = ros::Duration(1.0/publ_rate_);

  it_ = 0;
  it_ = new image_transport::ImageTransport(nh);
  depth_pub_ = it_->advertise("depth_output_throttle", 1);
  sub = nh_node.subscribe("/camera/depth/image", 1000, &DepthSubscriptionThrottle::callBackDepth, this);
}

void DepthSubscriptionThrottle::convert32FC1To8UC1(cv_bridge::CvImagePtr cv_ptr_depth, cv::Mat& depth_image_32FC1, cv::Mat& depth_image_8UC1)
{
  for(size_t i = 0; i < depth_image_32FC1.rows; i++)
  {
    float* Di = cv_ptr_depth->image.ptr<float>(i);
    float* Ii = depth_image_32FC1.ptr<float>(i);
    char* Ivi = depth_image_8UC1.ptr<char>(i);

    for(size_t j = 0; j < cv_ptr_depth->image.cols; j++)
    {
      if(Di[j] > 0.0f)
      {
        Ii[j] = Di[j];
        Ivi[j] = (char) (255*((Di[j])/(5.5))); // some suitable values.. => For visualization
      }

      else
      {
        Ii[j] = 0.0f;
        Ivi[j] = 0;
      }
    }
  }
}

void DepthSubscriptionThrottle::callBackDepth(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr_depth;

  try
  {
    cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // filter depth image - no negatives or NAN - (uchar) img_depth for visualization
  // depth_clean is the true depth in Meter
  cv::Mat depth_clean(cv_ptr_depth->image.rows, cv_ptr_depth->image.cols, CV_32FC1);//True depth
  cv::Mat img_depth(cv_ptr_depth->image.rows, cv_ptr_depth->image.cols, CV_8UC1);//For visualization
  convert32FC1To8UC1(cv_ptr_depth, depth_clean, img_depth);

  //cv::imshow("depth_imagedepth_image", img_depth);
  //cv::waitKey(3);

  cv_bridge::CvImage depth_msg;
  depth_msg.header   = cv_ptr_depth->header; // Same timestamp and tf frame as input image
  depth_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
  depth_msg.image    = img_depth;

  ros::Time now;
  now = ros::Time::now();
  if((now - g_last_time) > g_period)
  {
    depth_pub_.publish(depth_msg.toImageMsg());
    g_last_time = now;
  }
}


//
