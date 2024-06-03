#include "olny_cv/xyc_control.h"

Xyc_control::Xyc_control(const double Kp,const double Ki,const double Kd,const double speed)
:kp(Kp),ki(Ki),kd(Kd),speed(speed),_detector(940,50,26)
{
  this->debug = true;
  this->img_x_2 = 470.0;
  this->_prev_motor.angle = 0.0;
  this->_prev_motor.speed = 0.0;
  this->_pub_motor.speed = speed;
  prev_angles.fill(0.0);
  this->integral = 0.0;
  this->prev_angles_idx = 0;
}

Xyc_control::~Xyc_control()
{
}

xycar_msgs::xycar_motor Xyc_control::set_control(const cv::Mat& in_img){
  //set control foward, angular
  ROS_INFO("==== lane detect start ====");
  lane_img = _detector.window_search(in_img);

  cv::imshow("lane detection", lane_img);
  cv::waitKey(1);

  lane_res = _detector.get_value();
  
  // 레퍼런스 값 계산
  double reference = (lane_res[2][3].x + lane_res[2][4].x*(0.5) + lane_res[2][5].x*(0.5) + lane_res[2][6].x + lane_res[2][7].x*(1) + lane_res[2][8].x*(1.5) + lane_res[2][9].x*(1.5) + lane_res[2][10].x - 480 * 8) / 300.0;
  ROS_INFO("reference input: %f",reference);
  
  // I (Integral) 제어 계산
  double sum_diff = 0.0;
  for (size_t i = 0; i < prev_angles.size() - 1; ++i) {
    double diff = prev_angles[i + 1] - prev_angles[i];
    sum_diff += diff;
    // cout<<sum_diff<<",";
    // cout<<diff<<endl;
  }

  double avg_diff = sum_diff / (prev_angles.size() - 1);
  ROS_INFO("avg diff: %f",avg_diff);

  // I (Integral) 항 계산
  integral += avg_diff;

  // P (Proportional) 항 계산
  double proportional = kp * reference;
  ROS_INFO("p term: %f",proportional);

  // D (Derivative) 항 계산
  double derivative = kd * avg_diff;
  ROS_INFO("d term: %f",derivative);

  // I (Integral) 항 계산
  double integral_term = ki * integral;
  ROS_INFO("i term: %f",integral_term);

  // PID 제어 계산
  double control_output = proportional + integral_term + derivative;

  // 이전 각도 배열 갱신
  prev_angles[prev_angles_idx] = _pub_motor.angle;
  prev_angles_idx = (prev_angles_idx + 1) % prev_angles.size();
  ROS_INFO("idx: %d",prev_angles_idx);
  // 출력값 조정
  _pub_motor.angle = control_output;

  // 각도 범위 제한
  if(_pub_motor.angle > 0.3){
    _pub_motor.angle = 0.3;
  }
  if (_pub_motor.angle < -0.3){
    _pub_motor.angle = -0.3;
  }
  
  ROS_INFO("angular(published): %f", _pub_motor.angle);
  ROS_INFO("=====================");
  _pub_motor.speed = speed; // 속도 설정
  _prev_motor = _pub_motor;
  return _pub_motor;
}
