#include "ConvexShape.h"
#include <utility>
#include "Node.h"
#include "opencv2/opencv.hpp"
#include "Trigonometry.h"

ConvexShape::ConvexShape(std::vector<cv::Point> raw_points) :
    raw_points_(std::move(raw_points)) {

  //Order is important
  hull_ = calculateHull();
  if (isHullValid()) {
    point_angles_ = calculatePointAngles();
    calculateRotatedVector();
  }
}

bool ConvexShape::isShapeValid() {
  if (point_angles_rotated_.size() != 4) {
    return false;
  }

  return Node::filter(80, 100, point_angles_rotated_.at(0).angle) &&
      Node::filter(80, 100, point_angles_rotated_.at(1).angle) &&
      Node::filter(140, 160, point_angles_rotated_.at(2).angle) &&
      Node::filter(20, 40, point_angles_rotated_.at(3).angle);
}

PointVector ConvexShape::getHull() {
  return hull_;
}

PointAngleVector ConvexShape::getPointAngles() {
  return point_angles_;
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

PointVector ConvexShape::calculateHull() {
  PointVector hull{};
  cv::convexHull(raw_points_, hull, false);
  return hull;
}

PointAngleVector ConvexShape::calculatePointAngles() {
  PointAngleVector point_angles;

  cv::Point point_a = hull_.at(hull_.size() - 2);
  cv::Point point_b = hull_.at(hull_.size() - 1);

  for (auto &point_c : hull_) {
    double angle = Trigonometry::CalcAngleCTriangle(point_a, point_c, point_b);

    if (angle < 170 || angle > 190) {
      point_angles.push_back({point_b, angle});
    }

    point_a = point_b;
    point_b = point_c;
  }

  return point_angles;
}

bool ConvexShape::calculateRotatedVector() {
  point_angles_rotated_ = point_angles_;

  int count = 0;

  int pos;
  bool posSet = false;
  for (int i = 0; i < point_angles_rotated_.size(); i++) {
    if (Node::filter(80, 100, point_angles_rotated_.at(i).angle)) {
      count++;
    }

    if (count == 2 && !posSet) {
      posSet = true;
      pos = i;
    }
  }

  if (count != 2) {
    point_angles_rotated_.clear();
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(point_angles_rotated_.begin(), point_angles_rotated_.begin() + 1, point_angles_rotated_.end());
  }

  return true;
}


PointAngleVector ConvexShape::getRotatedPointAngles() {
  return point_angles_rotated_;
}

Point2fVector ConvexShape::getRotatedPointAngles2f() {
  Point2fVector points;
  points.reserve(point_angles_rotated_.size());

  std::transform(point_angles_rotated_.begin(),
                 point_angles_rotated_.end(),
                 std::back_inserter(points),
                 [](PointAngle &point_angle) { return point_angle.point; }
                 );

  return points;
}


