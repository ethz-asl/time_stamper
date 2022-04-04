#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"
#include "gtest/gtest.h"
#include "ConvexShape.h"
#include "Common.h"

class ConvexShapeTest {
 public:
  explicit ConvexShapeTest(int tolerance) : convex_shape_(tolerance) {}
  static int Filter(double min, double max, double value) {
    return Common::Filter(min, max, value);
  };
  ConvexShape convex_shape_;
 private:
};

ConvexShapeTest GetValidConvexShape() {
  ConvexShapeTest convex_shape_test(10);

  PointVector valid_points({
    {293, 338},
    {268, 338},
    {217, 338},
    {590, 337},
    {541, 337},
    {492, 337},
    {369, 337},
    {318, 337},
    {639, 337},
    {393, 337},
    {344, 269},
    {217, 268},
    {492, 269}
  });
  convex_shape_test.convex_shape_.Process(valid_points);
  return convex_shape_test;
}

ConvexShapeTest GetInvalidConvexShape() {
  ConvexShapeTest convex_shape_test(10);
  PointVector invalid_points({
    {123, 387},
    {419, 373},
    {464, 371},
    {506, 369},
    {546, 367},
    {583, 365},
    {618, 364},
    {682, 362},
    {791, 356},
    {815, 356},
    {626, 280},
    {342, 270},
    {188, 264}
  });

  convex_shape_test.convex_shape_.Process(invalid_points);
  return convex_shape_test;
}

TEST(ConvexShapeFilter, TestOptimalValue) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  EXPECT_EQ(convex_shape_test.Filter(10, 30, 10), true);
}

TEST(ConvexShapeFilter, TestLowerBound) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  EXPECT_EQ(convex_shape_test.Filter(10, 30, 10.0), true);
  EXPECT_EQ(convex_shape_test.Filter(10, 30, 9.9), false);
}

TEST(ConvexShapeFilter, TestUpperBound) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  EXPECT_EQ(convex_shape_test.Filter(10, 30, 30.0), true);
  EXPECT_EQ(convex_shape_test.Filter(10, 30, 30.1), false);
}

TEST(ConvexShapeToleranceFilter, TestOptimalValue) {
  ConvexShapeTest convex_shape_test(10);
  convex_shape_test.convex_shape_.isInRange(10, 10);
}

TEST(ConvexShapeToleranceFilter, TestLowerBound) {
  ConvexShapeTest convex_shape_test(10);
  ASSERT_EQ(convex_shape_test.convex_shape_.isInRange(10, 0), true);
  ASSERT_EQ(convex_shape_test.convex_shape_.isInRange(10, -0.1), false);
}

TEST(ConvexShapeToleranceFilter, TestUpperBound) {
  ConvexShapeTest convex_shape_test(10);
  ASSERT_EQ(convex_shape_test.convex_shape_.isInRange(10, 20), true);
  ASSERT_EQ(convex_shape_test.convex_shape_.isInRange(10, 20.1), false);
}

TEST(ConvexShapeValid, TestIsHullValid) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  ASSERT_EQ(convex_shape_test.convex_shape_.isHullValid(),true);
}

TEST(ConvexShapeValid, TestIsShapeValid) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  ASSERT_EQ(convex_shape_test.convex_shape_.isShapeValid(),true);
}

TEST(ConvexShapeValid, TestGetSortedPointAngles) {
  ConvexShapeTest convex_shape_test = GetValidConvexShape();
  PointAngleVector point_angle_vector = convex_shape_test.convex_shape_.getSortedPointAngles();
  ASSERT_EQ(point_angle_vector.size(), 4);
  ASSERT_FLOAT_EQ( point_angle_vector.at(0).angle, 90);
  ASSERT_FLOAT_EQ( point_angle_vector.at(1).angle, 89.791649);
  ASSERT_FLOAT_EQ( point_angle_vector.at(2).angle, 155.38379);
  ASSERT_FLOAT_EQ( point_angle_vector.at(3).angle, 24.990149);
}

TEST(ConvexShapeInvalid, TestIsHullValid) {
  ConvexShapeTest convex_shape_test = GetInvalidConvexShape();
  ASSERT_EQ(convex_shape_test.convex_shape_.isHullValid(),true);
}

TEST(ConvexShapeInvalid, TestIsShapeValid) {
  ConvexShapeTest convex_shape_test = GetInvalidConvexShape();
  ASSERT_EQ(convex_shape_test.convex_shape_.isShapeValid(), false);
}

TEST(ConvexShapeInvalid, TestGetSortedPointAngles) {
  ConvexShapeTest convex_shape_test = GetInvalidConvexShape();
  PointAngleVector point_angle_vector = convex_shape_test.convex_shape_.getSortedPointAngles();
  ASSERT_TRUE(point_angle_vector.empty());
}
#pragma clang diagnostic pop