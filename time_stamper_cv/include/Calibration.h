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
  explicit Calibration(const CalibrationConfig& cfg);
  cv_bridge::CvImage ProcessImage(const sensor_msgs::Image &image);
  void SetVisualization(bool visualization);
  ~Calibration();

 private:
  cv::Mat ConvertToCvImage();
  PointVector ConvertKeyPoints();
  void Visualize(const cv::Mat &visualization_mat, int number);
  void VisualizeCorners(cv::Mat visualization_mat, PointAngleVector corners);
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
