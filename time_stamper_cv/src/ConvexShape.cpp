#include "ConvexShape.h"
#include <utility>
#include "Node.h"
#include "opencv2/opencv.hpp"
#include "Trigonometry.h"

ConvexShape::ConvexShape(std::vector<cv::Point> raw_points) :
    raw_points_(std::move(raw_points)) {}

bool ConvexShape::validateVector(const std::vector<PointAngle> &angles) {
  return angles.size() == 4;
}

bool ConvexShape::validateAngles(const std::vector<PointAngle> &a) {
  return Node::filter(80, 100, a.at(0).angle) &&
      Node::filter(80, 100, a.at(1).angle) &&
      Node::filter(140, 160, a.at(2).angle) &&
      Node::filter(20, 40, a.at(3).angle);
}

bool ConvexShape::rotateVector(std::vector<PointAngle> *a) {
  if (!validateVector(*a)) {
    return false;
  }

  int count = 0;

  int pos;
  bool posSet = false;
  for (int i = 0; i < a->size(); i++) {
    if (Node::filter(80, 100, a->at(i).angle)) {
      count++;
    }

    if (count == 2 && !posSet) {
      posSet = true;
      pos = i;
    }
  }

  if (count != 2) {
    return false;
  }

  if (!Node::filter(80, 100, a->at(pos).angle) && !Node::filter(80, 100, a->at(pos - 1).angle)) {
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(a->begin(), a->begin() + 1, a->end());
  }

  return validateAngles(*a);
}

PointVector ConvexShape::getHull() {
  PointVector hull{};
  cv::convexHull(raw_points_, hull, false);
  return hull;
}



PointAngleVector ConvexShape::getPointAngles() {
  PointVector hull = getHull();
  PointAngleVector point_angles;

  cv::Point point_a = hull.at(hull.size() - 2);
  cv::Point point_b = hull.at(hull.size() - 1);

  for (auto &point_c : hull) {
    double angle = Trigonometry::CalcAngleCTriangle(point_a, point_c, point_b);

    if (angle < 170 || angle > 190) {
      point_angles.push_back({point_b, angle});
    }

    point_a = point_b;
    point_b = point_c;
  }

  return point_angles;
}
bool ConvexShape::isHullValid() {
  return getHull().size() >= 4;
}

Point2fVector ConvexShape::getVirtualCorners(int multiplier) {
  Point2fVector virtualCorners_multiplied;
  virtualCorners_multiplied.reserve(virtualCorners.size());

  std::transform(virtualCorners.begin(), virtualCorners.end(),
                 std::back_inserter(virtualCorners_multiplied),
                 [&multiplier](cv::Point2f &point){
                   return cv::Point2f(point.x * multiplier, point.y * multiplier);}
  );

  return virtualCorners_multiplied;
}

