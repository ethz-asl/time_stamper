#pragma once
#include "Common.h"
#include "opencv2/opencv.hpp"
#include "Configuration.h"

class LedParser {
 public:
  explicit LedParser(LedRowConfig  led_row_config, int image_crop_size = 16);
  static Point3fVector GenerateLedRow(const LedRowConfig& cfg);
  static cv::Point2f Normalize(const cv::Point3_<float>& pt);

  void ProcessImage(cv::Mat image);
  void TransformLedRow(const cv::Mat& homography);
  double GetLedBrightness(int index, float radius = 10.0f);
  bool isLedOn(int index, float radius = 10.0f, int min_brightness = 40);
  const Point3fVector &GetLedRow() const;
  int GetLedBinaryCounter();

  ~LedParser() = default;

 private:
  LedRowConfig led_row_config_;
  cv::Mat image_;
  int size_;
  Point3fVector led_row_{};
  cv::Mat kernel_normalized_;
};
