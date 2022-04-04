#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"
#include "trigonometry.h"

TEST(Trigonometry, TestCalcDistanceBasic) {
  cv::Point point_a{3, 0};
  cv::Point point_b{0, 0};

  ASSERT_FLOAT_EQ(Trigonometry::CalcDistance(point_a, point_b), 3);
}

TEST(Trigonometry, TestCalcInnerAngleBasic) {
  cv::Point point_a {0, 4};
  cv::Point point_b{3, 0};
  cv::Point point_c{0, 0};

  ASSERT_FLOAT_EQ(Trigonometry::CalcInnerAngle(point_a, point_b, point_c), 90);
  ASSERT_FLOAT_EQ(Trigonometry::CalcInnerAngle(point_b, point_a, point_c), 90);
}

TEST(Trigonometry, TestDistanceBasic1) {
  float a = 9;
  float b = 5;
  float c = 8;
  ASSERT_FLOAT_EQ(Trigonometry::LawOfCos(a, b, c), 62.181862);
}

TEST(Trigonometry, TestDistanceBasic2) {
  float a = 3;
  float b = 4;
  float c = 5;
  double d = Trigonometry::LawOfCos(a, b, c);
  double e = Trigonometry::LawOfCos(b, c, a);
  double f = Trigonometry::LawOfCos(a, c, b);

  ASSERT_DOUBLE_EQ(e+f, 90.0);
  ASSERT_DOUBLE_EQ(d+e+f, 180.0);
}
