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
  /**
   * @param keypoints
   * @return greatest and 2nd greatest value
   */
  static point2 GetLeftCornerLeds(const std::vector<cv::KeyPoint>& keypoints);

  /**
   *
   * @param keypoints
   * @return
   */
  static corner_leds CalcCornerLeds(const std::vector<cv::KeyPoint>& keypoints);
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};

};
