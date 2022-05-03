#pragma once

#include "sysfs/IGpioSubsystem.h"

class MockGpioSubsystem: public IGpioSubsystem {
 public:
  MOCK_METHOD0(isExported, bool());
  MOCK_METHOD0(exprt, bool());
  MOCK_METHOD0(unexport, bool());
  MOCK_METHOD0(isRunning, bool());
  MOCK_METHOD1(setDirection, bool(GPIO_DIRECTION gpio_direction));
  MOCK_METHOD1(setGpioMode, bool(GPIO_MODE gpio_mode));

  ~MockGpioSubsystem() override = default;
};
