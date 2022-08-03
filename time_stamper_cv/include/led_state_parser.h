#pragma once

#include "opencv2/opencv.hpp"

#include "common.h"
#include "configuration.h"

class LedStateParser {
 public:
  explicit LedStateParser(LedRowConfigRepository  led_row_config_, int image_crop_size = 16);

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
   * @param led_row_name LED row
   * @param homography
   */
  void transformLedRow(const std::string& led_row_name, const cv::Mat &homography);

  /**
   * Creates an (size_ * size_) sub-image and calculates average pixel brightness.
   * @param led_row_name LED row
   * @param index LED index
   * @return value between 0 and 255 or -1 on error.
   */
  double getLedBrightness(const std::string& led_row_name, int index) const;

  /**
   * Checks if LED at specific index in specified LED row is on.
   * @param led_row_name LED row
   * @param index LED row index
   * @param min_brightness Minimum brightness, value between 1 and 255.
   * @return true if average pixel brightness > min_brightness, otherwise false.
   */
  bool isLedOn(const std::string& led_row_name, int index, float min_brightness = 40.0) const;

  /**
   * Get the binary value of the specified LED row
   * @param led_row_name LED row
   * @return binary value
   */
  int getBinaryValue(const std::string& led_row_name) const;

  /**
   * Default destructor
   */
  ~LedStateParser() = default;

  /**
   * Public getter
   * @param led_row_name LED row
   * @return selected LED row
   */
  const Point3fVector &getLedRow(const std::string& led_row_name) const;

 private:
  LedRowConfigRepository led_row_configs_{};
  std::map<std::string, Point3fVector> led_rows_;
  cv::Mat image_{};
  int size_{};
  cv::Mat kernel_normalized_{};
};
