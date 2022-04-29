#pragma once
#include <iostream>
#include "SysfsPwm.h"
#include "ros/ros.h"

enum LedMode {
  FPS,
  EXPOSURE
};

class Node {
 public:
  /**
   * Creates new node with IPwmSubsystem
   * @param sysfs_pw
   * @param led_mode
   */
  explicit Node(IPwmSubsystem &sysfs_pwm, LedMode led_mode);

  /**
   * Node initialization.
   * @param frequency in hz
   * @param forceReset resets pwmchip if true
   * @return true if successful, otherwise false
   */
  bool Init(int frequency, bool forceReset);

  /**
   * Starts the node and loops until SIGINT is raised.
   */
  void Start();

  /**
   * Turns off pwmchip but does NOT unexport.
   */
  void CleanUp();

  /**
   * Default destructor.
   */
  ~Node() = default;

  /**
   * Used to catch csignal.
   */

  static bool run_node;
 private:
  bool SetGpioMode();

  LedMode mode_;
  ros::Publisher timestamp_pub_;
  ros::NodeHandle nh_;
  IPwmSubsystem &pwm_subsystem_;
};

