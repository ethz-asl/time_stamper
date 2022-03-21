#include "gtest/gtest.h"
#include "Node.h"
#include "ShapeValidation.h"


TEST(VectorRotation, TestInvalidValues) {
  std::vector<double> a{107.393, 77.4274, 20.864, 159.141};
  std::vector<double> out{};

  ASSERT_EQ(ShapeValidation::rotateVector(&a), false);
}

//Angles are valid but shape is inverted
TEST(VectorRotation, TestCounterclockwiseValues) {
  std::vector<double> a{93.7209, 86.0854, 25.9109, 155.71};
  std::vector<double> out{};

  ASSERT_EQ(ShapeValidation::rotateVector(&a), false);
}

TEST(VectorRotation, TestValidValuesWithOneRotation) {
  std::vector<double> a{26.5729, 89.2481, 89.7771, 154.402};
  std::vector<double> out{};

  bool res = ShapeValidation::rotateVector(&a);
  ASSERT_EQ(res, true);

  ASSERT_DOUBLE_EQ(a.at(0), 89.2481);
  ASSERT_DOUBLE_EQ(a.at(1), 89.7771);
  ASSERT_DOUBLE_EQ(a.at(2), 154.402);
  ASSERT_DOUBLE_EQ(a.at(3), 26.5729);
}

TEST(VectorRotation, TestValidValuesWithTwoRotations) {
  std::vector<double> a{154.402, 26.5729, 89.2481, 89.7771};
  std::vector<double> out{};

  bool res = ShapeValidation::rotateVector(&a);
  ASSERT_EQ(res, true);

  ASSERT_DOUBLE_EQ(a.at(0), 89.2481);
  ASSERT_DOUBLE_EQ(a.at(1), 89.7771);
  ASSERT_DOUBLE_EQ(a.at(2), 154.402);
  ASSERT_DOUBLE_EQ(a.at(3), 26.5729);
}