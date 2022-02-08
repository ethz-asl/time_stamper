#include <iostream>
#include <csignal>
#include "ros/ros.h"
#include "Node.h"

__attribute__((unused)) bool run_node = true;

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    ROS_INFO("Received sigint. Shutting down.");
    run_node = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper");
  SysfsPwm sysfs_pwm;
  Node node(sysfs_pwm);
  signal(SIGINT, SignalHandler);
  if (!node.Init()) {
    ROS_FATAL("Failed to initialize node.");
  }
  node.Start();
  return 0;
}