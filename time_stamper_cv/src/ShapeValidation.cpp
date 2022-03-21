#include "ShapeValidation.h"
#include "Node.h"

bool ShapeValidation::validateVector(const std::vector<double>& a) {
  return a.size() == 4;
}

bool ShapeValidation::validateAngles(const std::vector<double>& a) {
  return Node::filter(80, 100, a.at(0)) &&
  Node::filter(80, 100, a.at(1)) &&
  Node::filter(140, 160, a.at(2)) &&
  Node::filter(20, 40, a.at(3));
}

bool ShapeValidation::rotateVector(std::vector<double> *a) {
  if (!validateVector(*a)) {
    return false;
  }

  int count = 0;

  int pos;
  bool posSet = false;
  for (int i = 0; i < a->size(); i++) {
    if (Node::filter(80, 100, a->at(i))) {
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

  if (a->at(pos) > 80 && a->at(pos) < 100 && a->at(pos - 1) > 80, a->at(pos - 1) < 100) {
    std::cout << "valid angles found" << std::endl;
  } else {
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(a->begin(), a->begin() + 1, a->end());
  }

  return validateAngles(*a);
}