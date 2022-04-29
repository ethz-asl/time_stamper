#include "Node.h"
#include <cstring>
#include "time_stamper_ros/Timestamp.h"
#include "TimestampManager.h"
#include "SysfsGpio.h"
#include "Filesystem.h"
bool Node::run_node = true;

Node::Node(IPwmSubsystem &pwm_subsystem, LedMode led_mode)
    : pwm_subsystem_(pwm_subsystem), mode_(led_mode) {}

bool Node::Init(int frequency, bool forceReset) {
  if (!SetGpioMode()) {
    ROS_ERROR_STREAM("Set gpio mode failed: " << strerror(errno) << std::endl);
  };

  ROS_INFO("Mode set.");

  if (forceReset) {
    pwm_subsystem_.Reset();
    ROS_INFO("Reset pwm");
  }

  timestamp_pub_ = nh_.advertise<time_stamper_ros::Timestamp>("time_stamper/Timestamp", 1);
  if (pwm_subsystem_.IsExported()) {
    ROS_INFO("Pwm already exported");
  } else {
    if (!pwm_subsystem_.Export()) {
      ROS_ERROR("Failed to export pwm");
      return false;
    }
    ROS_INFO("Exported pwm");
  }

  if (!pwm_subsystem_.SetFrequency(frequency)) {
    ROS_ERROR_STREAM("Failed to set Frequency: " << strerror(errno));
    return false;
  }
  ROS_INFO_STREAM("Set frequency to " << frequency);

  if (pwm_subsystem_.IsRunning()) {
    ROS_WARN("Pwm already running");
  } else {
    ROS_INFO("Starting pwm");
    if (!pwm_subsystem_.Start()) {
      ROS_ERROR("Could not start pwm");
      return false;
    }
    ROS_INFO("Started pwm");
  }
  return true;
}

void Node::Start() {
  TimestampManager timestamp_manager;

  while (ros::ok() && Node::run_node) {
    ros::Rate loop_rate(10);

    if (timestamp_manager.Poll()) {
      time_stamper_ros::Timestamp msg;
      msg.timestamp = timestamp_manager.GetLastTimestamp();
      timestamp_pub_.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  CleanUp();
}

void Node::CleanUp() {
  ROS_INFO("Cleaning up node");
  if (!pwm_subsystem_.Stop()) {
    ROS_ERROR_STREAM("Failed to stop pwm: " << strerror(errno));
    return;
  }
  ROS_INFO("Stopped pwm");
}

bool Node::SetGpioMode() {
  Filesystem filesystem;
  SysfsGpio sysfs_gpio(2, filesystem);

  if (!sysfs_gpio.IsExported()) {
    if (!sysfs_gpio.Export()) {
      std::cout << strerror(errno) << std::endl;
      return false;
    }
    std::cout << "Exported GPIO" << std::endl;
  }

  GPIO_MODE gpio_mode;
  switch (mode_) {
    case EXPOSURE:
      gpio_mode = LOW;
      break;
    case FPS:
      gpio_mode = HIGH;
      break;
    default:
      return false;
  }
  if (!sysfs_gpio.SetDirection(OUT) || !sysfs_gpio.SetGpioMode(gpio_mode)) {
    return false;
  }
  return true;
}
