#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "ConvexShape.h"
#include "Calibration.h"
#include "Configuration.h"

class Calibration;
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
  CalibrationConfig GetConfiguration();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_{"~"};
  ros::Publisher img_pub_{};
  std::vector<std::string> labels{"Bottom Left", "Top Left", "Top Right", "Bottom Right"};
  Calibration* calibration_;
};
