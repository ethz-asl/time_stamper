#pragma once
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"
#include "Configuration.h"
#include "cv_bridge/cv_bridge.h"
#include "LedParser.h"
#include "Detector.h"

class Calibration {
 public:
  /**
   * Creates a new Calibration object.
   * @param cfg See Configuration.h for more details
   */
  explicit Calibration(const CalibrationConfig& cfg);

  /**
   * Process a ros image.
   * @param image
   * @returns cv_bride::Image which can be published in ROS after calling toImageMsg().
   */
  cv_bridge::CvImage ProcessImage(const sensor_msgs::Image &image);

  /**
   * If set to true a window for visualization is opened and updated every frame
   * @param visualization
   */
  void SetVisualization(bool visualization);

  /**
   * Default destructor
   */
  ~Calibration() = default;

 private:
  static cv::Mat ConvertToCvImage(const sensor_msgs::Image &image) ;
  void Visualize(const cv::Mat &visualization_mat, int number) const;
  void VisualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const;
  static void Log(const std::string&);

  std::shared_ptr<Detector> detector_;
  std::shared_ptr<ConvexShape> convex_shape_{};
  std::shared_ptr<LedParser> led_parser_{};

  bool visualization_{false};
  static constexpr const char *OPENCV_WINDOW{"Visualization"};
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
};
