#pragma once
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"
#include "Configuration.h"
#include "cv_bridge/cv_bridge.h"

class ConvexShape;

class Calibration {
 public:
  explicit Calibration(CalibrationConfig cfg);
  cv_bridge::CvImage ProcessImage(const sensor_msgs::Image& image);
  void SetVisualization(bool visualization);

 private:
  cv::Mat ConvertToCvImage();
  std::vector<cv::Point> ConvertKeyPoints();
  void Visualize(const cv::Mat& visualization_mat, ConvexShape convex_shape, int number);
  void VisualizeCorners(cv::Mat visualization_mat, std::vector<PointAngle> corners);
  void SetKeypointStatus();
  std::vector<cv::Point3f> GenerateLedRow(int led_gap_x, int led_gap_y, int amount, float multiplier);

  std::vector<cv::KeyPoint> keypoints_{};
  cv::Ptr<cv::SimpleBlobDetector> detector_;
  int tolerance_;
  bool visualization_ = false;
  sensor_msgs::Image image_;
  bool isKeypointsEmpty = false;
  bool lastShapeValid = false;
  static constexpr const char*  OPENCV_WINDOW = "Image window";
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
};
