#include <csignal>
#include "ros/ros.h"
#include "Node.h"
#include "sysfs/SysfsPwm.h"
#include "sysfs/SysfsGpio.h"
#include "Filesystem.h"

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    ROS_INFO("Received sigint. Shutting down.");
    Node::run_node = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper");

  ros::NodeHandle nh_private("~");
  int frequency = nh_private.param("frequency", 50);
  std::string pwmchip_path = nh_private.param("pwmchip_path", std::string("/sys/class/pwm/pwmchip0"));

  Filesystem filesystem;
  SysfsPwm sysfs_pwm(pwmchip_path, filesystem);
  SysfsGpio sysfs_gpio(2, filesystem);

  Node node(sysfs_pwm, sysfs_gpio, FPS);
  signal(SIGINT, SignalHandler);

  if (!node.Init(frequency, false)) {
    ROS_FATAL("Failed to initialize node.");
    return -1;
  }
  node.Start();
  return 0;
}