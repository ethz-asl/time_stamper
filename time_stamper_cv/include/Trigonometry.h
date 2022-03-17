#pragma once
#include "opencv2/opencv.hpp"

class Trigonometry {
 public:
  static double CalcDistance(const cv::Point &a, const cv::Point &b);

  static double CalcAngleCTriangle(const cv::Point &point_a, const cv::Point &point_b, const cv::Point &point_c);

  static double CalcAngleCTriangle(double a, double b, double c);
};
