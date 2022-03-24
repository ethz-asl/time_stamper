#pragma once
#include "opencv2/opencv.hpp"

class Trigonometry {
 public:
  static double CalcDistance(const cv::Point &a, const cv::Point &b);

  static double CalcInnerAngle(const cv::Point &a, const cv::Point &b, const cv::Point &target);

  static double LawOfCos(double a, double b, double c);
};
