#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"
#include "Trigonometry.h"

TEST(algorithm, GetLeftCornerLeds) {
// TODO Implement
/*Test values
X: 191.227 Y: 433.151
X: 327.372 Y: 428.2
X: 413.169 Y: 424.062
X: 496.617 Y: 420.542
X: 618.001 Y: 417.038
X: 696.365 Y: 415.483
X: 773.845 Y: 411.934
X: 887.715 Y: 409.041
X: 283.751 Y: 313.101
X: 323.83 Y: 312.743
X: 186.286 Y: 314.175
X: 656.664 Y: 306.679
186.286314.175
191.227433.151
*/
}

TEST(CalcDistance, Basic) {
  cv::Point point_a;
  point_a.x = 3;
  point_a.y = 0;
  cv::Point point_b;
  point_b.x = 0;
  point_b.y = 0;

  ASSERT_FLOAT_EQ(Trigonometry::CalcDistance(point_a, point_b), 3);
}


TEST(CalcAngleCTriangle, TestPointsBasic) {
  cv::Point point_a;
  point_a.x = 0;
  point_a.y = 5;
  cv::Point point_b;
  point_b.x = 3;
  point_b.y = 0;
  cv::Point point_c;
  point_c.x = 0;
  point_c.y = 0;

  ASSERT_FLOAT_EQ(Trigonometry::CalcAngleCTriangle(point_a, point_b, point_c), 90);
}

TEST(CalcAngleCTriangle, TestDistanceBasic) {

  float a = 9;
  float b = 5;
  float c = 8;


  ASSERT_FLOAT_EQ(Trigonometry::CalcAngleCTriangle(a, b, c), 62.181862);
}
