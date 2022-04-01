#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "Calibration.h"
#include "Configuration.h"

class Calibration;
class LedDetectionNode {
 public:
  LedDetectionNode();

  /**
   * Called when an image is published on a subscribed node.
   * @param image
   */
  void CallbackRawImage(const sensor_msgs::Image& image) const;

  /**
   * Initializes the node.
   */
  void Init();

  /**
   * Starts the node.
   */
  void Start();

  /**
   * Default destructor
   */
  ~LedDetectionNode() = default;

 private:
  CalibrationConfig GetConfiguration() const;

  ros::NodeHandle nh_{};
  ros::NodeHandle nh_private_{"~"};
  __attribute__((unused)) ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};
  std::shared_ptr<Calibration> calibration_{};
};
