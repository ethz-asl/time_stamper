#pragma once

#include <string>
#include <Filesystem.h>
#include <ISysfsSubsystem.h>

enum GPIO_MODE {
  HIGH,
  LOW
};

enum GPIO_DIRECTION{
  IN,
  OUT
};

class SysfsGpio : public ISysfsSubsystem {
 public:
  SysfsGpio(int gpio_nr, IFileSystem& file_system);
  bool IsExported() override;
  bool Export() override;
  bool Unexport() override;
  bool IsRunning() override;
  bool SetDirection(GPIO_DIRECTION direction);
  bool SetGpioMode(GPIO_MODE mode);
  ~SysfsGpio() override = default;

 private:
  //Not supported
  bool Start() override;
  bool Stop() override;

  inline static const std::string GPIO_PATH = "/sys/class/gpio";
  inline static const std::string GPIO = "/gpio";

  GPIO_DIRECTION direction_;
  const int gpio_nr_;
  std::string gpio_path_;
  IFileSystem &fs_;
};
