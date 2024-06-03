#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class LaneDetection {
public:
  LaneDetection(int img_x, int window_height, int nwindows);
  Mat window_search(const cv::Mat& binary_line);

  vector<vector<Point>> get_value();
  vector<vector<Point>> out_vec;
  
private:
  int img_x;
  int window_height;
  int nwindows;
  int nothing_left_x_base;
  int nothing_right_x_base;
  vector<int> nothing_pixel_left_x;
  vector<int> nothing_pixel_right_x;
  vector<int> nothing_pixel_y;

  vector<Point> left;
  vector<Point> right;
  vector<Point> center;

  void detect_nothing();
  Vec3f fitPoly(const std::vector<int>& y, const std::vector<int>& x, int order);
};

#endif // LANE_DETECTION_H
