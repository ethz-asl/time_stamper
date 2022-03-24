#pragma once
#include <vector>
#include <opencv2/core/types.hpp>
#include "Common.h"

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
  bool ToleranceFilter(int expected_value, double actual_value) const;
  ~ConvexShape() = default;

 private:
  void calculateHull();
  PointAngleVector calculatePointAngles();
  bool calculateRotatedVector();
  static bool Filter(double min, double max, double value);

  int tolerance_;
  PointVector raw_points_{};
  PointVector hull_{};
  PointAngleVector point_angles_sorted_{};

  Point2fVector virtualCorners{
    {0 , 0},
    {0, 17},
    {66, 17},
    {102, 0},
    };
};
