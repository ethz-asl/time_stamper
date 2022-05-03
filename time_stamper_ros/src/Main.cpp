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
  bool exposure_mode = nh_private.param("exposure_mode", false);
  int gpio_nr = nh_private.param("gpio_pin", 2);

  Filesystem fs;
  SysfsPwm sysfs_pwm(pwmchip_path, fs);
  SysfsGpio sysfs_gpio(gpio_nr, fs);

  Node node(sysfs_pwm, sysfs_gpio, exposure_mode ? EXPOSURE : FPS);
  signal(SIGINT, SignalHandler);

  if (!node.init(frequency, false)) {
    ROS_FATAL("Failed to initialize node.");
    return -1;
  }
  node.start();
  return 0;
}