#pragma once

#include <fcntl.h>
class IPwmSubsystem {
 public:
  IPwmSubsystem() = default;
  virtual bool IsExported() = 0;
  virtual bool Export() = 0;
  virtual bool Unexport() = 0;
  virtual bool IsRunning() = 0;
  virtual bool Start() = 0;
  virtual bool Stop() = 0;
  virtual bool SetFrequency(int hz) = 0;
  virtual bool ChangeDutyCycle(int hz) = 0;
  virtual bool ChangeDutyCycleRaw(int value) = 0;
  virtual bool Write(const std::string& path, const std::string& message) = 0;
  virtual bool Read(const std::string& path, void* buffer, size_t buffer_size) = 0;
  ~IPwmSubsystem() = default;
};
