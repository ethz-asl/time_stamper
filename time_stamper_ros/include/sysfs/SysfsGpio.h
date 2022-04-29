#pragma once

#include <string>
#include "Filesystem.h"
#include "sysfs/ISysfsSubsystem.h"
#include "IGpioSubsystem.h"

class SysfsGpio : public IGpioSubsystem {
 public:
  SysfsGpio(int gpio_nr, IFileSystem& file_system);
  bool IsExported() override;
  bool Export() override;
  bool Unexport() override;
  bool IsRunning() override;
  bool SetDirection(GPIO_DIRECTION direction) override;
  bool SetGpioMode(GPIO_MODE mode) override;
  ~SysfsGpio() override = default;

 private:
  bool Start() override;
  bool Stop() override;

  inline static const char* GPIO = "/gpio";

  GPIO_DIRECTION direction_;
  const int gpio_nr_;
  std::string gpio_path_;
  IFileSystem &fs_;
};
