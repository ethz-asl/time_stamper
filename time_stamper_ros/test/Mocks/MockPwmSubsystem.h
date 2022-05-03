#pragma once
#include "sysfs/IPwmSubsystem.h"

class MockPwmSubsystem : public IPwmSubsystem {
 public:
  MOCK_METHOD0(isExported, bool());
  MOCK_METHOD0(exprt, bool());
  MOCK_METHOD0(unexport, bool());
  MOCK_METHOD0(isRunning, bool());
  MOCK_METHOD0(start, bool());
  MOCK_METHOD0(stop, bool());
  MOCK_METHOD0(reset, bool());
  MOCK_METHOD1(setFrequency, bool(int hz));
  MOCK_METHOD1(changeDutyCycle, bool(int hz));
  MOCK_METHOD1(changeDutyCycleRaw, bool(int value));
  MOCK_METHOD2(getFrequency, bool(void *buffer, ssize_t size));
};