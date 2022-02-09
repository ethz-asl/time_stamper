#pragma once
#include <string>
#include <fcntl.h>
#include "IPwmSubsystem.h"

class SysfsPwm : public IPwmSubsystem {
 public:
  SysfsPwm() = default;
  bool IsExported() override;
  bool Export() override;
  bool Unexport() override;
  bool IsRunning() override;
  bool Start() override;
  bool Stop() override;
  bool SetFrequency(int hz) override;

  [[deprecated("This function is experimental and might break.")]]
  bool ChangeDutyCycle(int value) override;
  bool Write(const std::string& path, const std::string& message) override;
  bool Read(const std::string& path, void* buffer, size_t buffer_size) override;

  ~SysfsPwm() = default;
 private:
  static bool DirectoryExists(const char* path);
  bool ChangeDutyCycleRaw(int value) override;
};
