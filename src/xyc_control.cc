#include "olny_cv/xyc_control.h"

Xyc_control::Xyc_control()
:_detector(940,40,10)
{
  this->kp = 0.0;
  this->ki = 0.0;
  this->kd = 0.0;
  this->debug = true;
  this->power = 0.0;
  this->img_x_2 = 470.0;
}

Xyc_control::~Xyc_control()
{
}

xycar_msgs::xycar_motor Xyc_control::set_control(const cv::Mat& in_img){
  //set control foward, angular
  ROS_INFO("==== lane detect start ====");
  lane_img = _detector.window_search(in_img);

  cv::imshow("lane detection",lane_img);
  cv:waitKey(1);

  lane_res = _detector.get_value();
  // if (debug){
  //   for (const auto& row : lane_res) {
  //     for (const auto& point : row) {
  //       std::cout << "(" << point.x << ", " << point.y << ") ";
  //     }
  //     std::cout << std::endl;
  //   }
  // }

  // image_x : 940 

  // set speed
  // power += (lane_res[2][0].x - img_x_2) * 0.1;
  // power += (lane_res[2][1].x - img_x_2) * 0.3;
  power += (lane_res[2][2].x - img_x_2) * 0.0000007;
  power += (lane_res[2][3].x - img_x_2) * 0.0000007;
  power += (lane_res[2][4].x - img_x_2) * 0.0000007;
  power += (lane_res[2][5].x - img_x_2) * 0.0000003;
  power += (lane_res[2][6].x - img_x_2) * 0.0000002;
  std::cout<< power << endl;
  _pub_motor.angle = power/4000;
  _pub_motor.speed = 0.3;

  _prev_motor = _pub_motor;
  return _pub_motor;
}