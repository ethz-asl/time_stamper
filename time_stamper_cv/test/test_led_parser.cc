#include "gtest/gtest.h"
#include "led_parser.h"

TEST(GenerateLedRow, TestVectorSize) {
  LedRowConfig led_row_config{{0, 0}, {1, 0}, 16, 1};
  const Point3fVector& led_row = LedParser::generateLedRow(led_row_config);
  ASSERT_EQ(led_row.size(), 16);
}

TEST(GenerateLedRow, TestLedPositions) {
  LedRowConfig led_row_config{{0, 0}, {1, 0}, 5, 1};
  const Point3fVector& led_row = LedParser::generateLedRow(led_row_config);

  cv::Point3_<float> pt1{1, 0, 1};
  cv::Point3_<float> pt2{2, 0, 1};
  cv::Point3_<float> pt3{3, 0, 1};
  cv::Point3_<float> pt4{4, 0, 1};
  cv::Point3_<float> pt5{5, 0, 1};

  ASSERT_EQ(led_row.at(0), pt1);
  ASSERT_EQ(led_row.at(1), pt2);
  ASSERT_EQ(led_row.at(2), pt3);
  ASSERT_EQ(led_row.at(3), pt4);
  ASSERT_EQ(led_row.at(4), pt5);
}

TEST(GenerateLedRow, TestLedPositionsInclined) {

  LedRowConfig led_row_config{{0, 0}, {2, 1}, 5, 1};
  const Point3fVector& led_row = LedParser::generateLedRow(led_row_config);

  cv::Point3_<float> pt1{2, 1, 1};
  cv::Point3_<float> pt2{4, 2, 1};
  cv::Point3_<float> pt3{6, 3, 1};
  cv::Point3_<float> pt4{8, 4, 1};
  cv::Point3_<float> pt5{10, 5, 1};

  ASSERT_EQ(led_row.at(0), pt1);
  ASSERT_EQ(led_row.at(1), pt2);
  ASSERT_EQ(led_row.at(2), pt3);
  ASSERT_EQ(led_row.at(3), pt4);
  ASSERT_EQ(led_row.at(4), pt5);
}