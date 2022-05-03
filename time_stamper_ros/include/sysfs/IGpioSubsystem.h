#pragma once
#include "ISysfsSubsystem.h"

enum GPIO_MODE {
  HIGH,
  LOW
};

enum GPIO_DIRECTION {
  IN,
  OUT
};

class IGpioSubsystem : public ISysfsSubsystem {
 public:
  bool isExported() override = 0;
  bool exprt() override = 0;
  bool unexport() override = 0;
  bool isRunning() override = 0;
  SYSFS_NOT_SUPPORTED bool start() override {return false;}
  SYSFS_NOT_SUPPORTED bool stop() override {return false;}

  virtual bool setDirection(GPIO_DIRECTION gpio_direction) = 0;
  virtual bool setGpioMode(GPIO_MODE gpio_mode) = 0;
  ~IGpioSubsystem() override = default;
};
