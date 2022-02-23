#include <iostream>
#include <csignal>
#include "ros/ros.h"
#include "Node.h"

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    ROS_INFO("Received sigint. Shutting down.");
    Node::run_node = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper");
  SysfsPwm sysfs_pwm;
  Node node(sysfs_pwm);
  signal(SIGINT, SignalHandler);

  ros::NodeHandle nh_private("~");
  int frequency = nh_private.param("frequency", 50);

  if (!node.Init(frequency, false)) {
    ROS_FATAL("Failed to initialize node.");
  }
  node.Start();
  return 0;
}