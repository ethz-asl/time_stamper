#include "Node.h"
#include <cstring>
#include "time_stamper_ros/Timestamp.h"
#include "TimestampManager.h"
Node::Node(const SysfsPwm& sysfs_pwm)
: sysfs_pwm_(sysfs_pwm) {

}
bool Node::Init() {
  timestamp_pub_ = nh_.advertise<time_stamper_ros::Timestamp>("time_stamper/Timestamp", 1);
  if (sysfs_pwm_.IsExported()) {
    ROS_INFO("Pwm already exported");
  } else {
    if (!sysfs_pwm_._Export()) {
      ROS_ERROR("Failed to export pwm");
      return false;
    }
    ROS_INFO("Exported pwm");
  }

  int frequency = 50;
  if (!sysfs_pwm_.SetFrequency(frequency)) {
    ROS_ERROR("Failed to set Frequency");
    return false;
  }
  ROS_INFO_STREAM("Set frequency to " << frequency);

  if (sysfs_pwm_.IsRunning()) {
    ROS_INFO("Pwm already running");
  } else {
    ROS_INFO("Starting pwm");
    if (!sysfs_pwm_.Start()) {
      ROS_ERROR("Could not start pwm");
      return false;
    }
    ROS_INFO("Started pwm");
  }
  return true;
}

void Node::Start() {
  TimestampManager timestamp_manager;
  extern bool run_node;
  while (ros::ok() && run_node) {
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
  if (!sysfs_pwm_.Stop()) {
    ROS_ERROR("Failed to stop pwm");
  }
  ROS_INFO("Stopped pwm");
}