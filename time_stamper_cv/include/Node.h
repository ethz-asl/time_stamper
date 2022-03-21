#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

typedef std::pair<cv::KeyPoint, cv::KeyPoint> point2;

struct corner_leds{
  cv::KeyPoint upperLeft;
  cv::KeyPoint upperRight;
  cv::KeyPoint lowerLeft;
  cv::KeyPoint lowerRight;
};

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
  bool isKeypointsEmpty = false;
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};

};
