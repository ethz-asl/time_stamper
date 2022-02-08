#pragma once
#include <iostream>
#include "SysfsPwm.h"
class Node {
 public:
  explicit Node(const SysfsPwm& sysfs_pwm);
  bool Init();
  void Start();
  void CleanUp();
  ~Node() = default;
 private:
  SysfsPwm sysfs_pwm_;
};
