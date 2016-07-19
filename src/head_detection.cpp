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
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// head_detection_class
#include "head_depth/head_detection_class.h"

HeadDetection::HeadDetection(ros::NodeHandle nh) :	nh_node(nh)
{
  depth_sub_= nh_node.subscribe("depth_output_throttle", 1000, &HeadDetection::callBackDepth, this);
  haar_cascade_file = ros::package::getPath("head_depth") + "/file/haarcascades/haarcascade_range_multiview_5p_bg.xml";
  classifier_cascade.load(haar_cascade_file);
}

void HeadDetection::callBackDepth(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr_depth;

  try
  {
    cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_depth(cv_ptr_depth->image);
  classifier_cascade.detectMultiScale(img_depth, rect_heads, 1.1, 6, CV_HAAR_DO_CANNY_PRUNING, cvSize(20, 20));

/*
  if (rect_heads.size()!=0)
    printf("%zd face(s) are found.\n", rect_heads.size());

  for (int i = 0; i < rect_heads.size(); i++) {
    //Rect r = rect_heads[i];
    printf("a face is found at Rect(%d,%d,%d,%d).\n",
      rect_heads[i].x,
      rect_heads[i].y,
      rect_heads[i].width,
      rect_heads[i].height);
  }
*/

  for (unsigned int i = 0; i < rect_heads.size(); i++)
  {
    cv::rectangle(img_depth,
      cv::Point(rect_heads[i].x, rect_heads[i].y),
      cv::Point(rect_heads[i].x + rect_heads[i].width, rect_heads[i].y + rect_heads[i].height),
      cv::Scalar(255), 2);
    }

    cv::imshow("head detection in depth image", img_depth);
    cv::waitKey(3);
  }


//
