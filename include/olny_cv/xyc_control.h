#include "xycar_msgs/xycar_motor.h"
#include "olny_cv/lane_detection.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <iterator>

class Xyc_control
{
private:
  /* data */
  cv::Mat _in_img;
  cv::Mat lane_img;

  xycar_msgs::xycar_motor _prev_motor;
  xycar_msgs::xycar_motor _pub_motor;
  
  LaneDetection _detector;
  std::vector<std::vector<cv::Point>> lane_res;

  std::array<double, 20> prev_angles; // Circular buffer to store previous _pub_motor.angle values
  size_t prev_angles_idx;

  // PID
  double kp;
  double ki;
  double kd;

  // stanly
  double s_gain;
  double cte;
  double heading_error = 0.0;

  double speed;

  double power;
  double integral;
  double img_x_2;


  bool debug;
  bool stanly;
public:
  Xyc_control(const bool stanly,const double Kp,const double Ki,const double Kd,const double s_gain,const double speed);
  ~Xyc_control();

  xycar_msgs::xycar_motor set_control(const cv::Mat& in_img);
  xycar_msgs::xycar_motor set_control_stanly(const cv::Mat& in_img);
};
