#include "ros/ros.h"
#include "LedDetectionNode.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper_cv_node");
  LedDetectionNode node;
  node.Init();
  node.Start();
  ros::spin();
  return 0;
}