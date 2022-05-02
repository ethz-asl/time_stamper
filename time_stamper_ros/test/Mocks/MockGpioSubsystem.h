#pragma once

#include "sysfs/IGpioSubsystem.h"

class MockGpioSubsystem: public IGpioSubsystem {
 public:
  MOCK_METHOD0(IsExported, bool());
  MOCK_METHOD0(Export, bool());
  MOCK_METHOD0(Unexport, bool());
  MOCK_METHOD0(IsRunning, bool());
  MOCK_METHOD1(SetDirection, bool(GPIO_DIRECTION gpio_direction));
  MOCK_METHOD1(SetGpioMode, bool(GPIO_MODE gpio_mode));

  ~MockGpioSubsystem() override = default;
};
