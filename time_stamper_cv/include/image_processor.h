#pragma once
#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "convex_shape.h"
#include "configuration.h"
#include "led_state_parser.h"
#include "keypoint_detector.h"
#include "time_stamper_cv/Ledstate.h"

class ImageProcessor {
 public:
  /**
   * Creates a new ImageProcessor object.
   * @param cfg See configuration.h for more details
   */
  explicit ImageProcessor(const ImageProcessorConfig& cfg);

  /**
   * Process a ros image.
   * @param image
   * @returns cv_bride::Image which can be published in ROS after calling toImageMsg().
   */
  cv_bridge::CvImage process(const sensor_msgs::Image &image);

  /**
   * If set to true, a visualization window is opened and updated every frame.
   * @param visualization
   */
  void setVisualization(bool visualization);

  /**
   * Message to be published
   * @return Led state with set values
   */
  time_stamper_cv::Ledstate getLedStateMessage();

  /**
   * Default destructor
   */
  ~ImageProcessor() = default;

 private:
  static cv::Mat convertToCvImage(const sensor_msgs::Image &image) ;
  void visualize(const cv::Mat &visualization_mat) const;
  void visualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const;
  static void log(const std::string&);

  std::shared_ptr<KeyPointDetector> detector_;
  std::shared_ptr<ConvexShape> convex_shape_{};
  std::shared_ptr<LedStateParser> led_parser_{};
  ImageProcessorConfig cfg_;

  bool visualization_{false};
  static constexpr const char *OPENCV_WINDOW{"Visualization"};
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
};
