#pragma once
#include "opencv2/opencv.hpp"

class Common {
 public:
  static bool Filter(double min, double max, double value) {
    if (min > max) {
      return false;
    }
    return value >= min && value <= max;
  }
};

struct PointAngle {
  cv::Point point;
  double angle;
};

class PointVector : public std::vector<cv::Point> {
 public:
  PointVector() = default;

  explicit PointVector(const std::vector<cv::Point>& keypoints) {
    std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(*this));
  }

  explicit PointVector(std::vector<cv::KeyPoint> keypoints) {
    std::transform(keypoints.begin(),
                   keypoints.end(),
                   std::back_inserter(*this),
                   [](cv::KeyPoint &kp) { return kp.pt; });
  }
  ~PointVector() = default;
};

//Typedefs
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<cv::Point3f> Point3fVector;
typedef std::vector<PointAngle> PointAngleVector;