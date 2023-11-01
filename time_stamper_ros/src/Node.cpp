#include "Node.h"
#include <cstring>
#include "time_stamper_ros/Timestamp.h"
#include "TimestampManager.h"


bool Node::run_node = true;

Node::Node(IPwmSubsystem &pwm_subsystem, IGpioSubsystem &gpio_subsystem, LedMode led_mode)
    : pwm_subsystem_(pwm_subsystem), gpio_subsystem_(gpio_subsystem), mode_(led_mode) {
  server_.setCallback(boost::bind(&Node::configCallback, this, _1));
}

bool Node::init(int frequency, bool forceReset) {
  if (!setGpioMode()) {
    ROS_ERROR_STREAM("Set gpio mode failed: " << strerror(errno) << std::endl);
    return false;
  }
  ROS_INFO("Set GPIO mode");

  if (forceReset) {
    pwm_subsystem_.reset();
    ROS_INFO("Reset pwm");
  }

  timestamp_pub_ = nh_.advertise<time_stamper_ros::Timestamp>("time_stamper/Timestamp", 1);
  if (pwm_subsystem_.isExported()) {
    ROS_INFO("Pwm already exported");
  } else {
    if (!pwm_subsystem_.exprt()) {
      ROS_ERROR("Failed to export pwm");
      return false;
    }
    ROS_INFO("Exported pwm");
  }

  if (!pwm_subsystem_.setFrequency(frequency)) {
    ROS_ERROR_STREAM("Failed to set Frequency: " << strerror(errno));
    return false;
  }
  ROS_INFO_STREAM("Set frequency to " << frequency);

  if (pwm_subsystem_.isRunning()) {
    ROS_WARN("Pwm already running");
  } else {
    ROS_INFO("Starting pwm");
    if (!pwm_subsystem_.start()) {
      ROS_ERROR("Could not start pwm");
      return false;
    }
    ROS_INFO("Started pwm");
  }
  is_initialized_ = true;
  return true;
}

void Node::start() {
  TimestampManager timestamp_manager;

  while (ros::ok() && Node::run_node) {
    ros::Rate loop_rate(10);

    if (timestamp_manager.poll()) {
      time_stamper_ros::Timestamp msg;
      msg.timestamp = timestamp_manager.getLastTimestamp();
      timestamp_pub_.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  cleanUp();
}

void Node::cleanUp() {
  ROS_INFO("Cleaning up node");
  if (!pwm_subsystem_.stop()) {
    ROS_ERROR_STREAM("Failed to stop pwm: " << strerror(errno));
    return;
  }
  ROS_INFO("Stopped pwm");
}

bool Node::setGpioMode() {
  if (!gpio_subsystem_.isExported()) {
    if (!gpio_subsystem_.exprt()) {
      ROS_INFO_STREAM("GPIO exprt failed: " << strerror(errno));
      return false;
    }
    ROS_INFO("Exported GPIO");
  } else {
    ROS_INFO_ONCE("GPIO already exported");
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
      ROS_ERROR_STREAM("Invalid gpio mode: " << gpio_mode << std::endl);
      return false;
  }
  /* Used to toggle between Led modes. GPIO must always be output */
  if (!gpio_subsystem_.setDirection(OUT) || !gpio_subsystem_.setGpioMode(gpio_mode)) {
    return false;
  }
  return true;
}

void Node::configCallback(time_stamper_ros::LedConfig &config) {

  //Ignore if not initialized to avoid memory corruption
  if (!is_initialized_) {
    return;
  }
  mode_ = static_cast<LedMode>(config.mode); //Convert int to enum

  pwm_subsystem_.setFrequency(config.frequency);
  setGpioMode();
  ROS_INFO_STREAM("Config update received. Mode: " << mode_ << " Frequency: " << config.frequency);
}
