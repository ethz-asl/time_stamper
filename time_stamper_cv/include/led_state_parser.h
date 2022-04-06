#pragma once
#include "common.h"
#include "opencv2/opencv.hpp"
#include "configuration.h"

class LedStateParser {
 public:
  explicit LedStateParser(LedRowConfig led_row_config, int image_crop_size = 16);

  /**
   * Generate homogenous 2d coordinates from LedRowConfig.
   * Used for some matrix operations.
   * @param cfg See configuration.h
   */
  static Point3fVector generateLedRow(const LedRowConfig &cfg);

  /**
   * Converts point3f to point2f.
   * @param pt
   * @return normalized point
   */
  static cv::Point2f normalize(const cv::Point3_<float> &pt);

  /**
   * Processes new image. Overrides previous led_row.
   * @param image
   */
  void processImage(const cv::Mat &image);

  /**
   * Transforms LED row with homography.
   * @param homography
   */
  void transformLedRow(const cv::Mat &homography);

  /**
   * Creates an (size_ * size_) sub-image and calculates average pixel brightness.
   * @param index LED index
   * @return value between 0 and 255 or -1 on error.
   */
  double getLedBrightness(int index) const;

  /**
   * Checks if LED at specific index is on.
   * @param index LED row index
   * @param min_brightness Minimum brightness, value between 1 and 255.
   * @return true if average pixel brightness > min_brightness, otherwise false.
   */
  bool isLedOn(int index, float min_brightness = 40.0) const;

  /**
   * Get the binary value of the LED row
   * @return binary value
   */
  int getBinaryValue() const;

  /**
   * Default destructor
   */
  ~LedStateParser() = default;

  //!Public getter
  const Point3fVector &getLedRow() const;

 private:
  LedRowConfig led_row_config_{};
  cv::Mat image_{};
  int size_{};
  Point3fVector led_row_{};
  cv::Mat kernel_normalized_{};
};
