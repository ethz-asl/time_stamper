#pragma once

struct PointAngle {
  cv::Point point;
  double angle;
};

//Forward declarations
class ConvexShape;

//Typedefs
typedef std::vector<cv::Point> PointVector;
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<cv::Point3f> Point3fVector;
typedef std::vector<PointAngle> PointAngleVector;