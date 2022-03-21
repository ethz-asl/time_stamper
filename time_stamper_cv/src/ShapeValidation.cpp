#include "ShapeValidation.h"
#include "Node.h"

bool ShapeValidation::validateVector(const std::vector<PointAngle>& angles) {
  return angles.size() == 4;
}

bool ShapeValidation::validateAngles(const std::vector<PointAngle>& a) {
  return Node::filter(80, 100, a.at(0).angle) &&
  Node::filter(80, 100, a.at(1).angle) &&
  Node::filter(140, 160, a.at(2).angle) &&
  Node::filter(20, 40, a.at(3).angle);
}

bool ShapeValidation::rotateVector(std::vector<PointAngle> *a) {
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

  if (Node::filter(80, 100, a->at(pos).angle) && Node::filter(80, 100, a->at(pos - 1).angle)) {
  } else {
    return false;
  }

  for (int j = 0; j < pos - 1; j++) {
    std::rotate(a->begin(), a->begin() + 1, a->end());
  }

  return validateAngles(*a);
}