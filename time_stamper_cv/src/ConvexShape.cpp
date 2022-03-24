#include "ConvexShape.h"
#include <utility>
#include "Node.h"
#include "opencv2/opencv.hpp"
#include "Trigonometry.h"

ConvexShape::ConvexShape(int tolerance)
: tolerance_(tolerance) {}

void ConvexShape::Process(PointVector raw_points) {
  hull_.clear();
  point_angles_sorted_.clear();
  raw_points_ = std::move(raw_points);

  calculateHull();
  if (isHullValid()) {
    calculateRotatedVector();
  }
}

bool ConvexShape::isShapeValid() {
  if (point_angles_sorted_.size() != 4) {
    return false;
  }

  return Node::filter(80, 100, point_angles_sorted_.at(0).angle) &&
      Node::filter(80, 100, point_angles_sorted_.at(1).angle) &&
      Node::filter(140, 160, point_angles_sorted_.at(2).angle) &&
      Node::filter(20, 40, point_angles_sorted_.at(3).angle);
}

PointVector ConvexShape::getHull() {
  return hull_;
}

bool ConvexShape::isHullValid() {
  return hull_.size() >= 4;
}

Point2fVector ConvexShape::getVirtualCorners(int multiplier) {
  Point2fVector virtualCorners_multiplied;
  virtualCorners_multiplied.reserve(virtualCorners.size());

  std::transform(virtualCorners.begin(), virtualCorners.end(),
                 std::back_inserter(virtualCorners_multiplied),
                 [&multiplier](cv::Point2f &point) {
                   return cv::Point2f(point.x * multiplier, point.y * multiplier);
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

    if (angle < 170 || angle > 190) {
      point_angles.push_back({point_b, angle});
    }

    point_a = point_b;
    point_b = point_c;
  }

  return point_angles;
}

bool ConvexShape::calculateRotatedVector() {
  point_angles_sorted_ = calculatePointAngles();

  int count = 0;

  int pos;
  bool posSet = false;
  for (int i = 0; i < point_angles_sorted_.size(); i++) {
    if (Node::filter(80, 100, point_angles_sorted_.at(i).angle)) {
      count++;
    }

    if (count == 2 && !posSet) {
      posSet = true;
      pos = i;
    }
  }

  if (count != 2) {
    point_angles_sorted_.clear();
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(point_angles_sorted_.begin(), point_angles_sorted_.begin() + 1, point_angles_sorted_.end());
  }

  return true;
}


PointAngleVector ConvexShape::getSortedPointAngles() {
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
