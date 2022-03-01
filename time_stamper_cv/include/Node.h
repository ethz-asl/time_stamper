#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

class Node {
 public:
  Node();

  void CallbackRawImage(const sensor_msgs::Image& image);
  bool Init();
  void Start();
  ~Node();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};

};
