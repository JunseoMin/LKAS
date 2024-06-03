#include "olny_cv/lane_detection.h"
#include <numeric>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

LaneDetection::LaneDetection(int img_x, int window_height, int nwindows) {
  this->img_x = img_x;
  this->window_height = window_height;
  this->nwindows = nwindows;
  detect_nothing();
}

void LaneDetection::detect_nothing() {
  //in case no lane detected
  nothing_left_x_base = round(img_x * 0.140625);
  nothing_right_x_base = img_x - round(img_x * 0.140625);

  nothing_pixel_left_x.assign(nwindows, round(img_x * 0.140625));
  nothing_pixel_right_x.assign(nwindows, img_x - round(img_x * 0.140625));

  for (int index = 0; index < nwindows; ++index) {
    nothing_pixel_y.push_back(round(window_height / 2) * index);
  }
}

cv::Mat LaneDetection::window_search(const cv::Mat& binary_line) {
  // Check the number of channels in the input image
  cv::Mat gray_image;
  if (binary_line.channels() == 4) {
    cv::cvtColor(binary_line, gray_image, cv::COLOR_RGBA2GRAY);
  } else if (binary_line.channels() == 3) {
    cv::cvtColor(binary_line, gray_image, cv::COLOR_BGR2GRAY);
  } else if (binary_line.channels() == 1) {
    gray_image = binary_line;
  } else {
    throw std::runtime_error("Unsupported number of channels in input image");
  }

  // histogram
  int bottom_half_y = gray_image.rows / 2;
  cv::Mat hist;
  cv::reduce(gray_image(cv::Range(bottom_half_y, gray_image.rows), cv::Range::all()), hist, 0, cv::REDUCE_SUM, CV_32S);
  
  std::vector<int> histogram(hist.cols);
  for (int i = 0; i < hist.cols; ++i) {
    histogram[i] = hist.at<int>(0, i);
  }

  int midpoint = histogram.size() / 2;
  int left_x_base = std::distance(histogram.begin(), std::max_element(histogram.begin(), histogram.begin() + midpoint));
  int right_x_base = std::distance(histogram.begin(), std::max_element(histogram.begin() + midpoint, histogram.end()));

  if (left_x_base == 0) {
      left_x_base = nothing_left_x_base;
  }
  if (right_x_base == midpoint) {
      right_x_base = nothing_right_x_base;
  }
  
  cv::Mat out_img;
  cv::cvtColor(gray_image, out_img, cv::COLOR_GRAY2BGR);

  int margin = 80;
  int min_pix = round((margin * 2 * window_height) * 0.0031);

  std::vector<cv::Point> lane_pixels;
  cv::findNonZero(gray_image, lane_pixels);
  
  std::vector<int> lane_pixel_y, lane_pixel_x;
  for (auto& p : lane_pixels) {
      lane_pixel_y.push_back(p.y);
      lane_pixel_x.push_back(p.x);
  }

  std::vector<int> left_lane_idx, right_lane_idx;

  for (int window = 0; window < nwindows; ++window) {
    int win_y_low = gray_image.rows - (window + 1) * window_height;
    int win_y_high = gray_image.rows - window * window_height;

    int win_x_left_low = left_x_base - margin;
    int win_x_left_high = left_x_base + margin;
    int win_x_right_low = right_x_base - margin;
    int win_x_right_high = right_x_base + margin;

    if (left_x_base != 0) {
        cv::rectangle(out_img, cv::Point(win_x_left_low, win_y_low), cv::Point(win_x_left_high, win_y_high), cv::Scalar(0, 255, 0), 2);
    }
    if (right_x_base != midpoint) {
        cv::rectangle(out_img, cv::Point(win_x_right_low, win_y_low), cv::Point(win_x_right_high, win_y_high), cv::Scalar(0, 0, 255), 2);
    }

    for (int i = 0; i < lane_pixels.size(); ++i) {
      if (lane_pixel_y[i] >= win_y_low && lane_pixel_y[i] < win_y_high) {
        if (lane_pixel_x[i] >= win_x_left_low && lane_pixel_x[i] < win_x_left_high) {
          left_lane_idx.push_back(i);
        }
        if (lane_pixel_x[i] >= win_x_right_low && lane_pixel_x[i] < win_x_right_high) {
          right_lane_idx.push_back(i);
        }
      }
    }

    if (left_lane_idx.size() > min_pix) {
        left_x_base = std::accumulate(left_lane_idx.begin(), left_lane_idx.end(), 0, [&](int sum, int idx) { return sum + lane_pixel_x[idx]; }) / left_lane_idx.size();
    }
    if (right_lane_idx.size() > min_pix) {
        right_x_base = std::accumulate(right_lane_idx.begin(), right_lane_idx.end(), 0, [&](int sum, int idx) { return sum + lane_pixel_x[idx]; }) / right_lane_idx.size();
    }
  }

  std::vector<int> left_x, left_y, right_x, right_y;
  for (int idx : left_lane_idx) {
      left_x.push_back(lane_pixel_x[idx]);
      left_y.push_back(lane_pixel_y[idx]);
  }
  for (int idx : right_lane_idx) {
      right_x.push_back(lane_pixel_x[idx]);
      right_y.push_back(lane_pixel_y[idx]);
  }

  if (left_x.empty() && right_x.empty()) 
  {
    left_x = nothing_pixel_left_x;
    left_y = nothing_pixel_y;
    right_x = nothing_pixel_right_x;
    right_y = nothing_pixel_y;
  } 
  else 
  {
    if (left_x.empty()) {
      left_x = right_x;
      std::transform(left_x.begin(), left_x.end(), left_x.begin(), [this](int val) { return val - round(img_x / 2); });
      left_y = right_y;
    } else if (right_x.empty()) {
      right_x = left_x;
      std::transform(right_x.begin(), right_x.end(), right_x.begin(), [this](int val) { return val + round(img_x / 2); });
      right_y = left_y;
    }
  }

  // Fit polynomials to find the lane lines
  cv::Vec3f left_fit = fitPoly(left_y, left_x, 2);
  cv::Vec3f right_fit = fitPoly(right_y, right_x, 2);

  // Generate y values
  std::vector<double> plot_y(100);
  std::iota(plot_y.begin(), plot_y.end(), 0);
  std::transform(plot_y.begin(), plot_y.end(), plot_y.begin(), [&](double y) { return y * (gray_image.rows - 1) / 99; });

  // Calculate x values for fitted polynomials
  std::vector<double> left_fit_x(100), right_fit_x(100), center_fit_x(100);
  for (int i = 0; i < 100; ++i) {
      double y = plot_y[i];
      left_fit_x[i] = left_fit[0] * y * y + left_fit[1] * y + left_fit[2];
      right_fit_x[i] = right_fit[0] * y * y + right_fit[1] * y + right_fit[2];
      center_fit_x[i] = (left_fit_x[i] + right_fit_x[i]) / 2;
  }

  // Convert to points for polylines
  std::vector<cv::Point> left, right, center;
  for (int i = 0; i < 100; ++i) {
      left.emplace_back(left_fit_x[i], plot_y[i]);
      right.emplace_back(right_fit_x[i], plot_y[i]);
      center.emplace_back(center_fit_x[i], plot_y[i]);
  }

  cv::polylines(out_img, left, false, cv::Scalar(0, 0, 255), 5);
  cv::polylines(out_img, right, false, cv::Scalar(0, 255, 0), 5);

  out_vec.clear();
  out_vec.push_back(left);
  out_vec.push_back(right);
  out_vec.push_back(center);
  
  return out_img;
}

std::vector<std::vector<cv::Point>> LaneDetection::get_value() {
  return this->out_vec;
}

cv::Vec3f LaneDetection::fitPoly(const std::vector<int>& y, const std::vector<int>& x, int order) {
  cv::Mat X(y.size(), order + 1, CV_64F);
  cv::Mat Y(y.size(), 1, CV_64F);

  for (int i = 0; i < y.size(); ++i) {
    Y.at<double>(i, 0) = x[i];
    for (int j = 0; j <= order; ++j) {
      X.at<double>(i, j) = std::pow(y[i], j);
    }
  }

  cv::Mat coefficients;
  cv::solve(X, Y, coefficients, cv::DECOMP_SVD);

  return cv::Vec3f(coefficients.at<double>(0, 0), coefficients.at<double>(1, 0), coefficients.at<double>(2, 0));
}