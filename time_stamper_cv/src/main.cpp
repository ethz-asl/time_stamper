#include "ros/ros.h"

#include "led_detection_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper_cv_node");
  LedDetectionNode node;
  node.init();
  node.start();
  ros::spin();
  return 0;
}