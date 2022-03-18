#include "ros/ros.h"
#include "Node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cv_node");
  Node node;
  if (!node.Init()) {
    return -1;
  }
  node.Start();
  ros::spin();
  return 0;
}