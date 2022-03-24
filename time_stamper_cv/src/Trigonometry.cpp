#include "Trigonometry.h"

double Trigonometry::CalcDistance(const cv::Point &a, const cv::Point &b) {
  return sqrt(std::pow((a.x - b.x), 2) + (std::pow((a.y - b.y), 2)));
}

double Trigonometry::CalcInnerAngle(
    const cv::Point &a, const cv::Point &b, const cv::Point &target
) {
  double distance_c = CalcDistance(a, b);
  double distance_a = CalcDistance(b, target);
  double distance_b = CalcDistance(target, a);
  return LawOfCos(distance_a, distance_b, distance_c);
}

double Trigonometry::LawOfCos(double a, double b, double c) {
  return acos((c * c - (a * a + b * b)) / -(2 * a * b)) * 180 / M_PI;
}
