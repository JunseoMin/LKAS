#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <iostream>

class BirdEyeView
{
public:
    // Constructor
    BirdEyeView(/* args */);
    ~BirdEyeView();

    // Image filtering func
    void edge_filter(cv::Mat& edge_in_img);
    void color_filter(cv::Mat& color_in_img);

    cv::Mat _out_img;
    cv::Mat _in_img;
    
    cv::Mat bird_eye_generator(const cv::Mat in_img_);

    // sobel params
    cv::Mat fmag, mag;
    cv::Mat dx,dy;

    cv::Point2f src_vertics[4];
    cv::Point2f dst_vertics[4];

    cv::Mat perspective_mat;
private:
    
};
