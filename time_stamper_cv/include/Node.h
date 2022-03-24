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

 private:
  CalibrationConfig GetConfiguration();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_{"~"};
  ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};
  Calibration* calibration_;
};
