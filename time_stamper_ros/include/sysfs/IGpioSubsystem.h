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
  bool IsExported() override = 0;
  bool Export() override = 0;
  bool Unexport() override = 0;
  bool IsRunning() override = 0;
  SYSFS_NOT_SUPPORTED bool Start() override = 0;
  SYSFS_NOT_SUPPORTED bool Stop() override = 0;

  virtual bool SetDirection(GPIO_DIRECTION gpio_direction) = 0;
  virtual bool SetGpioMode(GPIO_MODE gpio_mode) = 0;
  ~IGpioSubsystem() override = default;
};
