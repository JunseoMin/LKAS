#include "olny_cv/xyc_control.h"

Xyc_control::Xyc_control(const bool stanly,const double Kp,const double Ki,const double Kd,const double s_gain,const double speed)
:stanly(stanly),kp(Kp),ki(Ki),kd(Kd),s_gain(s_gain),speed(speed),_detector(940,50,26)
{
  this->debug = true;
  this->img_x_2 = 470.0;
  this->_prev_motor.angle = 0.0;
  this->_prev_motor.speed = 0.0;
  this->_pub_motor.speed = speed;
  prev_angles.fill(0.0);
  this->integral = 0.0;
  this->prev_angles_idx = 0;
  this->s_gain = s_gain;

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
  double reference = (lane_res[2][3].x*(0.7) + lane_res[2][4].x*(0.7) + lane_res[2][5].x*(0.7) + lane_res[2][6].x*(1.3) + lane_res[2][7].x*(1.3) + lane_res[2][8].x*(1.3) + lane_res[2][9].x + lane_res[2][10].x - 480 * 8) / 400.0;
  ROS_INFO("reference input: %f",reference);
  
  // I (Integral) 제어 계산
  double sum_diff = 0.0;
  
  for (size_t i = 0; i < prev_angles.size() - 1; ++i) {
    double diff = prev_angles[i + 1] - prev_angles[i];
    sum_diff += diff;
  }

  double avg_diff = sum_diff / (prev_angles.size() - 1);
  ROS_INFO("avg diff: %f",avg_diff);

  // I (Integral) term
  integral += sum_diff;

  // P (Proportional) term
  double proportional = kp * reference;
  ROS_INFO("p term: %f",proportional);
  // D (Derivative) term

  double derivative = kd * avg_diff;
  ROS_INFO("d term: %f",derivative);

  // I (Integral) term
  double integral_term = ki * integral;
  ROS_INFO("i term: %f",integral_term);

  // PID
  double control_output = proportional + integral_term + derivative;

  // 이전 각도 배열 갱신
  prev_angles[prev_angles_idx] = _pub_motor.angle;
  prev_angles_idx = (prev_angles_idx + 1) % prev_angles.size();
  ROS_INFO("idx: %d",prev_angles_idx);

  _pub_motor.angle = control_output;

  if(_pub_motor.angle > 0.3){
    _pub_motor.angle = 0.3;
  }
  if (_pub_motor.angle < -0.3){
    _pub_motor.angle = -0.3;
  }

  ROS_INFO("angular(published): %f", _pub_motor.angle);
  ROS_INFO("linear(published): %f", _pub_motor.speed);
  ROS_INFO("=====================");
  // _pub_motor.speed = speed; // 속도 설정
  _pub_motor.speed = speed;
  
  _prev_motor = _pub_motor;
  return _pub_motor;
}

xycar_msgs::xycar_motor Xyc_control::set_control_stanly(const cv::Mat& in_img){
  //set control foward, angular
  ROS_INFO("==== Stanly start ====");
  lane_img = _detector.window_search(in_img);

  cv::imshow("lane detection", lane_img);
  cv::waitKey(1);

  lane_res = _detector.get_value();
  
  double lane_angle = atan2(lane_res[2][3].y - lane_res[2][4].y, lane_res[2][3].x - lane_res[2][4].x);
  double lane_angle_degrees = lane_angle * 180 / M_PI;

  double psi_term = lane_angle - M_PI / 2;
  
  while (psi_term > M_PI) {
    psi_term -= 2 * M_PI;
  }
  while (psi_term < -M_PI) {
    psi_term += 2 * M_PI;
  }
  
  // lane center - image_x center pixel
  double error_pixel = (lane_res[2][3].x*(0.7) + lane_res[2][4].x*(0.7) + lane_res[2][5].x*(0.7) + lane_res[2][6].x*(1.3) + lane_res[2][7].x*(1.3) + lane_res[2][8].x*(1.3) + lane_res[2][9].x + lane_res[2][10].x - 480 * 8);

  ROS_INFO("reference: %f", error_pixel);
  
  double cte = error_pixel / 940.0; //normalize
  
  double control_output = psi_term + atan2(s_gain * cte, speed);

  // Set the motor angle
  _pub_motor.angle = control_output;
  ROS_INFO("angular(original): %f", control_output);

  // Constrain the angle to the vehicle's steering limits
  if (_pub_motor.angle > 0.3) {
    _pub_motor.angle = 0.3;
  }
  if (_pub_motor.angle < -0.3) {
    _pub_motor.angle = -0.3;
  }

  ROS_INFO("angular(published): %f", _pub_motor.angle);
  ROS_INFO("linear(published): %f", _pub_motor.speed);
  ROS_INFO("=====================");
  return _pub_motor;
}
