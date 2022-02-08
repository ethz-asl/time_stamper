#pragma once
#include <iostream>
#include "SysfsPwm.h"
#include "ros/ros.h"
class Node {
 public:
  explicit Node(const SysfsPwm& sysfs_pwm);
  bool Init();
  void Start();
  void CleanUp();
  ~Node() = default;
 private:
  ros::Publisher timestamp_pub_;
  ros::NodeHandle nh_;
  SysfsPwm sysfs_pwm_;
};

