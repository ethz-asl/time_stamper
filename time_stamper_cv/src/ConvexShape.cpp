#include "ConvexShape.h"
#include <utility>
#include "Node.h"
#include "opencv2/opencv.hpp"
#include "Trigonometry.h"

ConvexShape::ConvexShape(std::vector<cv::Point> raw_points) :
    raw_points_(std::move(raw_points)) {

  //Order is important
  hull_ = calculateHull();
  if (hull_.size() >= 4) {
    point_angles_ = calculatePointAngles();
  }
}

bool ConvexShape::validateInnerAngleSize() {
  return point_angles_.size() == 4;
}

bool ConvexShape::validateInnerAngles() {
  return Node::filter(80, 100, point_angles_rotated_.at(0).angle) &&
      Node::filter(80, 100, point_angles_rotated_.at(1).angle) &&
      Node::filter(140, 160, point_angles_rotated_.at(2).angle) &&
      Node::filter(20, 40, point_angles_rotated_.at(3).angle);
}

bool ConvexShape::rotateVector() {
  if (!validateInnerAngleSize()) {
    return false;
  }
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

  if (!Node::filter(80, 100, point_angles_rotated_.at(pos).angle) && !Node::filter(80, 100, point_angles_rotated_.at(pos - 1).angle)) {
    point_angles_rotated_.clear();
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(point_angles_rotated_.begin(), point_angles_rotated_.begin() + 1, point_angles_rotated_.end());
  }

  if(!validateInnerAngles()) {
    point_angles_rotated_.clear();
    return false;
  }
  return true;
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

PointAngleVector ConvexShape::getRotatedPointAngles() {
  return point_angles_rotated_;
}

