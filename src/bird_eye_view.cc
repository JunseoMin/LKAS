#include "olny_cv/bird_eye_view.h"

BirdEyeView::BirdEyeView(){
  ROS_INFO("===== Bird Eye View Initialize =====");

  // raw(y) col(x)
  // set target points
  src_vertics[0] = cv::Point2f(390,345);  //top left
  src_vertics[1] = cv::Point2f(566,345);  //top right
  src_vertics[2] = cv::Point2f(666,476);  //bottom right
  src_vertics[3] = cv::Point2f(290,476);  //bottom left
  
  dst_vertics[0] = cv::Point2f(375,0);  //dst bottom left 565
  dst_vertics[1] = cv::Point2f(565,0);  //dst bottom right
  dst_vertics[2] = cv::Point2f(565,479);  //dst top left
  dst_vertics[3] = cv::Point2f(375,479);  //dst top right

  perspective_mat = cv::getPerspectiveTransform(src_vertics, dst_vertics);
  _out_img = cv::Mat(480,960,CV_8UC1);
}

BirdEyeView::~BirdEyeView(){
}

void BirdEyeView::edge_filter(cv::Mat& edge_in_img){
  // using cv filter
  cv::Sobel(edge_in_img, dx, CV_32FC1, 1, 0);
  cv::Sobel(edge_in_img, dy, CV_32FC1, 0, 1);
  cv::magnitude(dx, dy, fmag);
  // fmag.convertTo(mag, CV_8UC3);

  edge_in_img = fmag > 50;
}


cv::Mat BirdEyeView::bird_eye_generator(cv::Mat in_img_){
  cv::resize(in_img_,in_img_,cv::Point2i(960,480),cv::INTER_LINEAR);  //8UC3
  cv::cvtColor(in_img_,_out_img,CV_8UC1);

  cv::warpPerspective(in_img_,in_img_,perspective_mat,cv::Point2i(960,480));
  cv::imshow("color birdeye",in_img_);
  ROS_INFO("==== image converted ====");  

  // IMG FILT SIZE : 500*290smoothed_angle
  // ORIGINAL : 800 * 600
  BirdEyeView::edge_filter(_out_img); // get edge filtered converted image
                                      // _out_img 8UC1 image 
  cv::warpPerspective(_out_img,_out_img,perspective_mat,cv::Point2i(960,480));

  return _out_img;
}

