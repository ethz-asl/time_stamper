#pragma once
#include "sysfs/IPwmSubsystem.h"

class MockPwmSubsystem : public IPwmSubsystem {
 public:
  MOCK_METHOD0(IsExported, bool());
  MOCK_METHOD0(Export, bool());
  MOCK_METHOD0(Unexport, bool());
  MOCK_METHOD0(IsRunning, bool());
  MOCK_METHOD0(Start, bool());
  MOCK_METHOD0(Stop, bool());
  MOCK_METHOD0(Reset, bool());
  MOCK_METHOD1(SetFrequency, bool(int hz));
  MOCK_METHOD1(ChangeDutyCycle, bool(int hz));
  MOCK_METHOD1(ChangeDutyCycleRaw, bool(int value));
  MOCK_METHOD2(GetFrequency, bool(void *buffer, ssize_t size));
};