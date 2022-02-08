#pragma once

class IPwmSubsystem {
 public:
  IPwmSubsystem() = default;
  virtual bool isExported() = 0;
  virtual bool _export() = 0;
  virtual bool unexport() = 0;
  virtual bool isRunning() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool changeFrequency() = 0;
  virtual bool changeDutyCycle(int hz) = 0;
  virtual bool changeDutyCycleRaw(int value) = 0;
  ~IPwmSubsystem() = default;
};
