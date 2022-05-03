#pragma once

#include <string>
#include "Filesystem.h"
#include "sysfs/ISysfsSubsystem.h"
#include "IGpioSubsystem.h"

class SysfsGpio : public IGpioSubsystem {
 public:
  SysfsGpio(int gpio_nr, IFileSystem& file_system);
  bool isExported() override;
  bool exprt() override;
  bool unexport() override;
  bool isRunning() override;
  bool setDirection(GPIO_DIRECTION gpio_direction) override;
  bool setGpioMode(GPIO_MODE gpio_mode) override;
  ~SysfsGpio() override = default;

 private:
  bool start() override;
  bool stop() override;

  inline static const char* GPIO = "/gpio";

  GPIO_DIRECTION direction_;
  const int gpio_nr_;
  std::string gpio_path_;
  IFileSystem &fs_;
};
