#include "xycar_msgs/xycar_motor.h"
#include "olny_cv/lane_detection.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>

class Xyc_control
{
private:
  /* data */
  cv::Mat _in_img;
  cv::Mat lane_img;

  xycar_msgs::xycar_motor _prev_motor;
  xycar_msgs::xycar_motor _pub_motor;
  
  LaneDetection _detector;
public:
  Xyc_control(/* args */);
  ~Xyc_control();

  xycar_msgs::xycar_motor set_control(const cv::Mat& in_img);
};
