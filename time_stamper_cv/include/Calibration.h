#pragma once
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"
#include "Configuration.h"
#include "cv_bridge/cv_bridge.h"

class ConvexShape;

class Calibration {
 public:
  explicit Calibration(CalibrationConfig cfg);
  cv_bridge::CvImage ProcessImage(const sensor_msgs::Image &image);
  void SetVisualization(bool visualization);
  int GetLedCounter(const cv::Mat &input_mat, cv::Mat visualization_mat);
  ~Calibration();

 private:
  cv::Mat ConvertToCvImage();
  PointVector ConvertKeyPoints();
  void Visualize(const cv::Mat &visualization_mat, int number);
  void VisualizeCorners(cv::Mat visualization_mat, PointAngleVector corners);
  void SetKeypointStatus();
  void SetShapeStatus();
  Point3fVector GenerateLedRow(
      const cv::Point2f &first_Led_Pos, const cv::Vec2i &next_led, int amount, float multiplier = 1);

  std::vector<cv::KeyPoint> keypoints_{};
  cv::Ptr<cv::SimpleBlobDetector> detector_;
  bool visualization_ = false;
  sensor_msgs::Image image_;
  bool isKeypointsEmpty = false;
  bool lastShapeValid = false;
  static constexpr const char *OPENCV_WINDOW = "Image window";
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
  ConvexShape *convex_shape_;
};
