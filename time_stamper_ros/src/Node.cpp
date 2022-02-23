#include "Node.h"
#include <cstring>
#include "time_stamper_ros/Timestamp.h"
#include "TimestampManager.h"
bool Node::run_node = true;

Node::Node(IPwmSubsystem& pwm_subsystem)
: pwm_subsystem_(pwm_subsystem) {

}

bool Node::Init(int frequency, bool forceReset) {
  if (forceReset) {
    pwm_subsystem_.Reset();
    ROS_INFO("Reset pwm");
    usleep(1e3 * 10);
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
    ROS_ERROR("Failed to set Frequency");
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
    ROS_ERROR("Failed to stop pwm");
  }
  ROS_INFO("Stopped pwm");
}