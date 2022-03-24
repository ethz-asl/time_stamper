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
  explicit ConvexShape(int tolerance);

  void Process(PointVector raw_points);
  PointVector getHull();
  bool isHullValid();
  Point2fVector getVirtualCorners(int multiplier = 1);
  PointAngleVector getSortedPointAngles();
  Point2fVector getPhysicalCorners();

  bool isShapeValid();
  ~ConvexShape() = default;

 private:
  void calculateHull();
  PointAngleVector calculatePointAngles();
  bool calculateRotatedVector();

  int tolerance_;
  PointVector raw_points_{};
  PointVector hull_{};
  PointAngleVector point_angles_sorted_{};

  std::vector<cv::Point2f> virtualCorners{
    {0 , 0},
    {0, 17},
    {66, 17},
    {102, 0},
    };
};
