#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "Calibration.h"
#include "Configuration.h"

class Calibration;
class Node {
 public:
  Node();

  /**
   * Called when an image is published on a subscribed node.
   * @param image
   */
  void CallbackRawImage(const sensor_msgs::Image& image);

  /**
   * Initializes the node.
   * @return true if successful, otherwise false.
   */
  bool Init();

  /**
   * Starts the node.
   */
  void Start();
  ~Node();

 private:
  CalibrationConfig GetConfiguration();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_{"~"};
  __attribute__((unused)) ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};
  Calibration* calibration_;
};
