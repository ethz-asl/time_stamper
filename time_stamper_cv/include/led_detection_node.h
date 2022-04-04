#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "image_processor.h"
#include "configuration.h"

class ImageProcessor;
class LedDetectionNode {
 public:
  LedDetectionNode();

  /**
   * Called when an image is published on a subscribed node.
   * @param image
   */
  void callbackRawImage(const sensor_msgs::Image& image) const;

  /**
   * Initializes the node.
   */
  void init();

  /**
   * Starts the node.
   */
  void start();

  /**
   * Default destructor
   */
  ~LedDetectionNode() = default;

 private:
  ImageProcessorConfig getConfiguration() const;

  ros::NodeHandle nh_{};
  ros::NodeHandle nh_private_{"~"};
  __attribute__((unused)) ros::Subscriber img_sub_{};
  ros::Publisher img_pub_{};
  std::shared_ptr<ImageProcessor> calibration_{};
};
