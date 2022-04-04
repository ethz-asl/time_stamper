#include "Trigonometry.h"

double Trigonometry::CalcDistance(const cv::Point &a, const cv::Point &b) {
  return cv::norm(a-b);
}

double Trigonometry::CalcInnerAngle(
    const cv::Point &a, const cv::Point &b, const cv::Point &target) {
  double distance_ab = CalcDistance(a, b);
  double distance_bc = CalcDistance(b, target);
  double distance_ca = CalcDistance(target, a);
  return LawOfCos(distance_bc, distance_ca, distance_ab);
}

double Trigonometry::LawOfCos(double a, double b, double c) {
  if (a == 0) {
    std::cout << "Trigonometry::LawOfCos() failed: a must not be 0" << std::endl;
    return 1;
  } else if (b == 0) {
      std::cout << "Trigonometry::LawOfCos() failed: b must not be 0" << std::endl;
    return 1;
  }
  return acos((c * c - (a * a + b * b)) / -(2 * a * b)) * 180 / M_PI;
}
