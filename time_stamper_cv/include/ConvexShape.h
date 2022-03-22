#pragma once
#include <vector>
#include <opencv2/core/types.hpp>

struct PointAngle {
  cv::Point point;
  double angle;
};
#include "Node.h"

typedef std::vector<cv::Point> PointVector;
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<PointAngle> PointAngleVector;

class ConvexShape {
 public:
  explicit ConvexShape(std::vector<cv::Point> raw_points);

  PointVector getHull();
  bool isHullValid();
  PointAngleVector getPointAngles();
  Point2fVector getVirtualCorners(int multiplier = 1);

  static bool rotateVector(std::vector<PointAngle> *a);
  static bool validateAngles(const std::vector<PointAngle>& a);
  static bool validateVector(const std::vector<PointAngle>& a);
  ~ConvexShape() = default;

 private:
  PointVector raw_points_{};

  std::vector<cv::Point2f> virtualCorners{
    {0 , 0},
    {0, 17},
    {66, 17},
    {102, 0},
    };
};
