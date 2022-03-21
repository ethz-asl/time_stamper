#pragma once
#include <vector>
#include <opencv2/core/types.hpp>

struct PointAngle {
  cv::Point point;
  double angle;
};
class ShapeValidation {
 public:
  static bool rotateVector(std::vector<PointAngle> *a);
  static bool validateAngles(const std::vector<PointAngle>& a);
  static bool validateVector(const std::vector<PointAngle>& a);
};
