#pragma once
#include <iostream>
#include "SysfsPwm.h"
#include "ros/ros.h"
class Node {
 public:
  explicit Node(IPwmSubsystem& sysfs_pwm);
  bool Init();
  void Start();
  void CleanUp();
  ~Node() = default;

  static bool run_node;
 private:
  ros::Publisher timestamp_pub_;
  ros::NodeHandle nh_;
  IPwmSubsystem& pwm_subsystem_;
};

