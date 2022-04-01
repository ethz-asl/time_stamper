#pragma once
#include "opencv2/opencv.hpp"

struct PointAngle {
  cv::Point point;
  double angle;
};

class PointVector : public std::vector<cv::Point> {
 public:
  PointVector() = default;
  explicit PointVector(std::vector<cv::KeyPoint> keypoints) {
    std::transform(keypoints.begin(),
                   keypoints.end(),
                   std::back_inserter(*this),
                   [](cv::KeyPoint &kp) { return kp.pt; });
  }
};

//Typedefs
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<cv::Point3f> Point3fVector;
typedef std::vector<PointAngle> PointAngleVector;