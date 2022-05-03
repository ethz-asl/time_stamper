#include "sysfs/SysfsGpio.h"


SysfsGpio::SysfsGpio(int gpio_nr, IFileSystem& file_system) :
    fs_(file_system), gpio_nr_(gpio_nr) {
    gpio_path_ = SYSFS_PATH + std::string(GPIO) + GPIO + std::to_string(gpio_nr);
}

bool SysfsGpio::isExported() {
  return fs_.directoryExists((gpio_path_ + "/").c_str());
}

bool SysfsGpio::exprt() {
  return fs_.write(SYSFS_PATH + std::string(SYSFS_EXPORT), std::to_string(gpio_nr_));
}

bool SysfsGpio::unexport() {
  return fs_.write(SYSFS_PATH + std::string(SYSFS_UNEXPORT), std::to_string(gpio_nr_));
}

bool SysfsGpio::isRunning() {
  //There is no "running" state for gpios, consider export state as running state
  return isExported();
}

bool SysfsGpio::start() {
  //Not supported
  return false;
}

bool SysfsGpio::stop() {
  //Not supported
  return false;
}

bool SysfsGpio::setDirection(GPIO_DIRECTION gpio_direction) {
  std::string direction;
  switch (gpio_direction) {
    case IN:
      direction = "in";
      break;
    case OUT:
      direction = "out";
      break;
    default:
      return false;
  }
  return fs_.write(gpio_path_ + "/direction", direction);
}

bool SysfsGpio::setGpioMode(GPIO_MODE gpio_mode) {
  if (direction_ == IN) {
    return false;
  }
  std::string mode;
  switch (gpio_mode) {
    case HIGH:
      mode = "1";
      break;
    case LOW:
      mode = "0";
      break;
    default:
      return false;
  }

  return fs_.write(gpio_path_ + "/value", mode);
}


