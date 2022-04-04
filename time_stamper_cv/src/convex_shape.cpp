#include "convex_shape.h"
#include <utility>
#include "opencv2/opencv.hpp"
#include "trigonometry.h"

ConvexShape::ConvexShape(int tolerance)
: tolerance_(tolerance) {}

void ConvexShape::process(const PointVector& raw_points) {
  hull_.clear();
  point_angles_sorted_.clear();
  raw_points_ = raw_points;

  calculateHull();
  if (isHullValid()) {
    calculateSortedPointAngles();
  }
}

bool ConvexShape::isInRange(double value1, double value2) const {
  return common::filter(value1 - tolerance_, value1 + tolerance_, value2);
}

bool ConvexShape::isShapeValid() const {
  if (point_angles_sorted_.size() != 4) {
    return false;
  }

  return isInRange(90, point_angles_sorted_.at(0).angle) &&
      isInRange(90, point_angles_sorted_.at(1).angle) &&
      isInRange(150, point_angles_sorted_.at(2).angle) &&
      isInRange(30, point_angles_sorted_.at(3).angle);
}

PointVector ConvexShape::getHull() const {
  return hull_;
}

bool ConvexShape::isHullValid() const {
  return hull_.size() >= 4;
}

Point2fVector ConvexShape::getVirtualCorners(float scaling) {
  Point2fVector virtualCorners_multiplied;
  virtualCorners_multiplied.reserve(virtualCorners.size());

  std::transform(virtualCorners.begin(), virtualCorners.end(),
                 std::back_inserter(virtualCorners_multiplied),
                 [&scaling](cv::Point2f &point) {
                   return cv::Point2f(point.x * scaling, point.y * scaling);
                 }
  );

  return virtualCorners_multiplied;
}

void ConvexShape::calculateHull() {
  cv::convexHull(raw_points_, hull_, false);
}

PointAngleVector ConvexShape::calculatePointAngles() {
  PointAngleVector point_angles;

  cv::Point point_a = hull_.at(hull_.size() - 2);
  cv::Point point_b = hull_.at(hull_.size() - 1);

  for (auto &point_c : hull_) {
    double angle = Trigonometry::CalcInnerAngle(point_a, point_c, point_b);

    //Sort out points on lines
    if (angle < (180 - tolerance_)) {
      point_angles.push_back({point_b, angle});
    }

    point_a = point_b;
    point_b = point_c;
  }

  return point_angles;
}

bool ConvexShape::calculateSortedPointAngles() {
  point_angles_sorted_ = calculatePointAngles();

  int count = 0;

  int pos;
  bool posSet = false;
  for (int i = 0; i < point_angles_sorted_.size(); i++) {
    if (isInRange(90, point_angles_sorted_.at(i).angle)) {
      count++;
    }

    if (count == 2 && !posSet) {
      posSet = true;
      pos = i;
    }
  }

  // Pattern not found
  if (count != 2) {
    point_angles_sorted_.clear();
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(point_angles_sorted_.begin(), point_angles_sorted_.begin() + 1, point_angles_sorted_.end());
  }

  return true;
}


PointAngleVector ConvexShape::getSortedPointAngles() const {
  return point_angles_sorted_;
}

Point2fVector ConvexShape::getPhysicalCorners() {
  Point2fVector points;
  points.reserve(point_angles_sorted_.size());

  std::transform(point_angles_sorted_.begin(),
                 point_angles_sorted_.end(),
                 std::back_inserter(points),
                 [](PointAngle &point_angle) { return point_angle.point; }
                 );

  return points;
}

void ConvexShape::pollShapeStatus(const std::function<void(std::string)> &function) {
  if (isShapeValid() != is_last_shape_valid_) {
    std::string v = isShapeValid() ? "Valid" : "Invalid";
    is_last_shape_valid_ = isShapeValid();
    function("Shape " + v);
  }
}

cv::Mat ConvexShape::getInvHomography() {
  return cv::findHomography(getPhysicalCorners(), getVirtualCorners(1)).inv();
}
