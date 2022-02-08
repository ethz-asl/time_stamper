#include <iostream>

#include "ros/ros.h"
#include "Node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper");
  Node node;
  if (!node.Init()) {
    ROS_FATAL("Failed to initialize node.");
  }
  SysfsPwm sysfs_pwm(1);
  sysfs_pwm.isRunning();
  return 0;
}