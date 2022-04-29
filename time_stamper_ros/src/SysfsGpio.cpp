#include "SysfsGpio.h"


SysfsGpio::SysfsGpio(int gpio_nr, IFileSystem& file_system) :
    fs_(file_system), gpio_nr_(gpio_nr) {
    gpio_path_ = GPIO_PATH + GPIO + std::to_string(gpio_nr);
}

bool SysfsGpio::IsExported() {
  return fs_.DirectoryExists((gpio_path_ + "/").c_str());
}

bool SysfsGpio::Export() {
  return fs_.Write(GPIO_PATH + SYSFS_EXPORT, std::to_string(gpio_nr_));
}

bool SysfsGpio::Unexport() {
  return fs_.Write(GPIO_PATH + SYSFS_UNEXPORT, std::to_string(gpio_nr_));
}

bool SysfsGpio::IsRunning() {
  //There is no "running" state for gpios, consider export state as running state
  return IsExported();
}

bool SysfsGpio::Start() {
  //Not supported
  return false;
}

bool SysfsGpio::Stop() {
  //Not supported
  return false;
}

bool SysfsGpio::SetDirection(GPIO_DIRECTION gpio_direction) {
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
  return fs_.Write(gpio_path_ + "/direction", direction);
}

bool SysfsGpio::SetGpioMode(GPIO_MODE gpio_mode) {
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

  return fs_.Write(gpio_path_ + "/value", mode);
}


