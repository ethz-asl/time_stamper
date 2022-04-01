#pragma once
#include "Common.h"
#include "opencv2/opencv.hpp"
#include "Configuration.h"

class LedParser {
 public:
  explicit LedParser(LedRowConfig led_row_config, int image_crop_size = 16);

  /**
   * Generate a point3f vector from LedRowConfig, where the z value is 1.
   * Used for some matrix operations.
   * @param cfg See Configuration.h
   */
  static Point3fVector GenerateLedRow(const LedRowConfig &cfg);

  /**
   * Converts point3f to point2f.
   * @param pt
   * @return normalized point
   */
  static cv::Point2f Normalize(const cv::Point3_<float> &pt);

  /**
   * Processes new image. Overrides previous led_row.
   * @param image
   */
  void ProcessImage(const cv::Mat &image);

  /**
   * Transforms LED row with homography.
   * @param homography
   */
  void TransformLedRow(const cv::Mat &homography);

  /**
   * Creates an (size_ * size_) sub-image and calculates average pixel brightness.
   * @param index LED index
   * @return value between 0 and 255 or -1 on error.
   */
  double GetLedBrightness(int index) const;

  /**
   * Checks if LED at specific index is on.
   * @param index LED row index
   * @param min_brightness Minimum brightness, value between 1 and 255.
   * @return true if average pixel brightness > min_brightness, otherwise false.
   */
  bool isLedOn(int index, int min_brightness = 40) const;

  /**
   * Get the binary value of the LED row
   * @return
   */
  int GetBinaryValue() const;

  /**
   * Default destructor
   */
  ~LedParser() = default;

  //!Public getter
  const Point3fVector &GetLedRow() const;

 private:
  LedRowConfig led_row_config_;
  cv::Mat image_;
  int size_;
  Point3fVector led_row_{};
  cv::Mat kernel_normalized_;
};
