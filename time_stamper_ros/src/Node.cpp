#include "Node.h"
#include <unistd.h>
#include <cstring>
#include "ros/ros.h"
#include "fcntl.h"
Node::Node(const SysfsPwm& sysfs_pwm)
: sysfs_pwm_(sysfs_pwm) {

}

void Node::Start() {
  extern bool run_node;
  while (ros::ok() && run_node) {
    ros::spinOnce();
  }
  CleanUp();
}

bool Node::Init() {
  //TODO Check if exported;

  if (sysfs_pwm_.IsRunning()) {
    ROS_INFO("Pwm already running");
  } else {
    ROS_INFO("Starting pwm");
    if (!sysfs_pwm_.Start()) {
      ROS_ERROR("Could not start pwm");
    }
  }


  /*
    int fd = open("/sys/kernel/time_stamper/ts_buffer", O_RDWR);
    if (fd == -1) {
      std::cout << "Error %i from open: " << errno << " " << strerror(errno) << std::endl;
      return false;
    }

      int buffer_size = 4096;
      while (true) {
        unsigned char temp_buffer[buffer_size];
        ssize_t n = read(fd, &temp_buffer, buffer_size);
        if (n > 0) {
          std::cout << "Read bytes: " << n << std::endl;
          for (int i = 0; i < n; i++) {
            std::cout << temp_buffer[i];
          }
          std::cout << std::endl;
        }
      }*/

}
void Node::CleanUp() {
  ROS_INFO("Cleaning up node");
}