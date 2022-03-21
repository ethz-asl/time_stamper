#include "gtest/gtest.h"
#include "ShapeValidation.h"

TEST(VectorRotation, TestInvalidValues) {

  //Ignore Point values
  std::vector<PointAngle> a{
      {{0, 0}, 107.393},
      {{0, 0}, 77.4274},
      {{0, 0}, 20.864},
      {{0, 0}, 159.141},
  };

  ASSERT_EQ(ShapeValidation::rotateVector(&a), false);
}


TEST(VectorRotation, TestCounterclockwiseValues) {
  //Angles are valid but shape is inverted
  //Ignore Point values
  std::vector<PointAngle> a{
      {{0, 0}, 93.7209},
      {{0, 0}, 86.0854},
      {{0, 0}, 25.9109},
      {{0, 0}, 155.71}
  };

  ASSERT_EQ(ShapeValidation::rotateVector(&a), false);
}

TEST(VectorRotation, TestValidValuesWithOneRotation) {
  //Ignore Point values
  std::vector<PointAngle> a{
    {{0, 0}, 26.5729},
    {{0, 0}, 89.2481},
    {{0, 0}, 89.7771},
    {{0, 0}, 154.402}
  };

  bool res = ShapeValidation::rotateVector(&a);
  ASSERT_EQ(res, true);

  ASSERT_DOUBLE_EQ(a.at(0).angle, 89.2481);
  ASSERT_DOUBLE_EQ(a.at(1).angle, 89.7771);
  ASSERT_DOUBLE_EQ(a.at(2).angle, 154.402);
  ASSERT_DOUBLE_EQ(a.at(3).angle, 26.5729);
}

TEST(VectorRotation, TestValidValuesWithTwoRotations) {
  //Ignore Point values
  std::vector<PointAngle> a{
    {{0, 0}, 154.402},
    {{0, 0}, 26.5729},
    {{0, 0}, 89.2481},
    {{0, 0}, 89.7771}
  };
  std::vector<double> out{};

  bool res = ShapeValidation::rotateVector(&a);
  ASSERT_EQ(res, true);

  ASSERT_DOUBLE_EQ(a.at(0).angle, 89.2481);
  ASSERT_DOUBLE_EQ(a.at(1).angle, 89.7771);
  ASSERT_DOUBLE_EQ(a.at(2).angle, 154.402);
  ASSERT_DOUBLE_EQ(a.at(3).angle, 26.5729);
}