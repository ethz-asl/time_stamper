#pragma once
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"
#include "Configuration.h"
#include "cv_bridge/cv_bridge.h"
#include "LedParser.h"

class ConvexShape;

//TODO refactoring
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
  ~Calibration();

 private:
  cv::Mat ConvertToCvImage() const;
  PointVector ConvertKeyPoints();
  void Visualize(const cv::Mat &visualization_mat, int number) const;
  void VisualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const;
  void SetKeypointStatus();
  void SetShapeStatus();

  std::vector<cv::KeyPoint> keypoints_{};
  cv::Ptr<cv::SimpleBlobDetector> detector_;
  bool visualization_ = false;
  sensor_msgs::Image image_;
  bool isKeypointsEmpty = false;
  bool isLastShapeValid = false;
  static constexpr const char *OPENCV_WINDOW = "Image window";
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};

  ConvexShape *convex_shape_;
  LedParser *led_parser_;
};
