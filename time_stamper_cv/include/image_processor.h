#pragma once
#include "opencv2/opencv.hpp"
#include "convex_shape.h"
#include "configuration.h"
#include "cv_bridge/cv_bridge.h"
#include "led_parser.h"
#include "detector.h"

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
   * Default destructor
   */
  ~ImageProcessor() = default;

 private:
  static cv::Mat convertToCvImage(const sensor_msgs::Image &image) ;
  void visualize(const cv::Mat &visualization_mat, int number) const;
  void visualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const;
  static void log(const std::string&);

  std::shared_ptr<detector> detector_;
  std::shared_ptr<ConvexShape> convex_shape_{};
  std::shared_ptr<LedParser> led_parser_{};

  bool visualization_{false};
  static constexpr const char *OPENCV_WINDOW{"Visualization"};
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
};
