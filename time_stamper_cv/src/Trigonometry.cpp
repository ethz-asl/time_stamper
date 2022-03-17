#include "Trigonometry.h"

double Trigonometry::CalcDistance(const cv::Point &a, const cv::Point &b) {
  return sqrt(std::pow((a.x - b.x), 2) + (std::pow((a.y - b.y), 2)));
}

double Trigonometry::CalcAngleCTriangle(
    const cv::Point &point_a, const cv::Point &point_b, const cv::Point &point_c
) {
  double distance_c = CalcDistance(point_a, point_b); //c between a - b
  double distance_a = CalcDistance(point_b, point_c); //a between b - c
  double distance_b = CalcDistance(point_c, point_a); //b between c - a
  return CalcAngleCTriangle(distance_a, distance_b, distance_c);
}

double Trigonometry::CalcAngleCTriangle(double a, double b, double c) {
  return acos((c * c - (a * a + b * b)) / -(2 * a * b)) * 180 / M_PI;
}
