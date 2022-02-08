#pragma once
#include "IPwmSubsystem.h"
class SysfsPwm : public IPwmSubsystem {
 public:
  explicit SysfsPwm(int test);
  bool isExported() override;
  bool _export() override;
  bool unexport() override;
  bool isRunning() override;
  bool start() override;
  bool stop() override;
  bool changeFrequency() override;
  bool changeDutyCycle(int hz) override;
  bool changeDutyCycleRaw(int value) override;
  ~SysfsPwm() = default;
 private:
  int test_;
};
