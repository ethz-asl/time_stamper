#pragma once
#include "Common.h"
#include "opencv2/opencv.hpp"

struct LedRowConfig {
  cv::Point2f first_led_pos;
  cv::Vec2i next_led;
  int amount;
  int multiplier = 1;
};

class LedParser {
 public:
  LedParser(const LedRowConfig& led_row_config, cv::Mat image, int image_crop_size = 16);
  static Point3fVector GenerateLedRow(const LedRowConfig& cfg);
  static cv::Point2f Normalize(const cv::Point3_<float>& pt);

  void TransformLedRow(const cv::Mat& homography);
  double GetLedBrightness(int index, float radius = 10.0f);
  bool isLedOn(int index, float radius = 10.0f, int min_brightness = 40);
  const Point3fVector &GetLedRow() const;
  int GetLedBinaryCounter();

  ~LedParser() = default;

 private:
  cv::Mat image_;
  int size_;
  Point3fVector led_row_{};
  cv::Mat kernel_normalized_;
};
