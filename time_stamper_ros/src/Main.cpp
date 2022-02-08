#include <iostream>
#include <csignal>
#include "ros/ros.h"
#include "Node.h"

__attribute__((unused)) bool run_node = true;

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    run_node = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper");
  SysfsPwm sysfs_pwm;
  Node node(sysfs_pwm);
  if (!node.Init()) {
    ROS_FATAL("Failed to initialize node.");
  }
  return 0;
}