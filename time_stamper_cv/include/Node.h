#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"


class Node {
 public:
  Node();

  void CallbackRawImage(const sensor_msgs::Image& image);
  bool Init();
  void Start();
  ~Node();

  static bool filter(double min, double max, double value);
  static constexpr const char*  OPENCV_WINDOW = "Image window";

 private:
  static std::vector<cv::Point> convertKeyPoints(std::vector<cv::KeyPoint>&);
  static std::vector<cv::Point2f> convertPointAngles(std::vector<PointAngle>&);
  static cv::Ptr<cv::SimpleBlobDetector> createBlobDetectorPtr();

  void visualizeCorners(cv::Mat visualization_mat, std::vector<PointAngle> corners);

  bool isKeypointsEmpty = false;
  bool isShapeValid = false;
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
};
