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
  PointAngleVector getRotatedPointAngles();
  Point2fVector getRotatedPointAngles2f();

  /**
   * Move both rectangular angles to vec.begin() without changing vector order.
   * @return
   */

  bool isShapeValid();
  ~ConvexShape() = default;

 private:
  PointVector calculateHull();
  PointAngleVector calculatePointAngles();
  bool calculateRotatedVector();

  PointVector raw_points_{};
  PointVector hull_{};
  PointAngleVector point_angles_{};
  PointAngleVector point_angles_rotated_{};

  std::vector<cv::Point2f> virtualCorners{
    {0 , 0},
    {0, 17},
    {66, 17},
    {102, 0},
    };
};
