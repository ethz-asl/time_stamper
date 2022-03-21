#pragma once
#include <vector>

class ShapeValidation {
 public:
  static bool rotateVector(std::vector<double> *a);
  static bool validateAngles(const std::vector<double>& a);
  static bool validateVector(const std::vector<double>& a);
};
