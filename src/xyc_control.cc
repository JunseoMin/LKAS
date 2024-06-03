#include "olny_cv/xyc_control.h"

Xyc_control::Xyc_control()
:_detector(940,40,10)
{
}

Xyc_control::~Xyc_control()
{
}

xycar_msgs::xycar_motor Xyc_control::set_control(const cv::Mat& in_img){
  //set control foward, angular
  ROS_INFO("==== lane detect start ====");
  lane_img = _detector.window_search(in_img);

  cv::imshow("lane detection",lane_img);
  cv:waitKey(0);
  
  _detector.get_value();
  

  _prev_motor = _pub_motor;
  return _pub_motor;
}