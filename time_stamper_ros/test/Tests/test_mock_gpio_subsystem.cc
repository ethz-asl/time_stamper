#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "MockPwmSubsystem.h"
#include "MockGpioSubsystem.h"
#include "Node.h"

using ::testing::Return;

/**
 * Helper function to expect default GpioSubsystem calls.
 * @param mock_pwm_subsystem
 */
void ExpectDefaultPwmSubsystem(MockPwmSubsystem &mock_pwm_subsystem) {
  EXPECT_CALL(mock_pwm_subsystem, isExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, setFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, isRunning()).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, start()).WillOnce(Return(true));
}

TEST(MockGpioSubsystem, TestNodeInitWhenUninitalized) {
  //Precondition: MockGpioSubsystem is not exported.
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultPwmSubsystem(mock_pwm_subsystem);

  //Expectation: Export and set direction to OUT and GPIO mode to HIGH.
  EXPECT_CALL(mock_gpio_subsystem, isExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_gpio_subsystem, exprt()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setDirection(OUT)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setGpioMode(HIGH)).Times(1).WillOnce(Return(true));

  node.init(50);
}

TEST(MockGpioSubsystem, TestNodeInitAlreadyExported) {
  //Precondition: MockGpioSubsystem is already exported.
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultPwmSubsystem(mock_pwm_subsystem);

  //Expectation: Set direction to OUT and GPIO mode to HIGH.
  EXPECT_CALL(mock_gpio_subsystem, isExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setDirection(OUT)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setGpioMode(HIGH)).Times(1).WillOnce(Return(true));

  node.init(50);
}

TEST(MockGpioSubsystem, TestNodeInitWithExposureMode) {
  //Precondition: MockGpioSubsystem is already exported. Node is created in exposure mode.
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, EXPOSURE);

  ExpectDefaultPwmSubsystem(mock_pwm_subsystem);

  //Expectation: Set direction to OUT and GPIO mode to LOW.
  EXPECT_CALL(mock_gpio_subsystem, isExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setDirection(OUT)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, setGpioMode(LOW)).Times(1).WillOnce(Return(true));

  node.init(50);
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
TEST(MockGpioSubsystem, TestStartNotSupported) {
  MockGpioSubsystem mock_gpio_subsystem;
  ASSERT_EQ(mock_gpio_subsystem.start(), false);
}

TEST(MockGpioSubsystem, TestStopNotSupported) {
  MockGpioSubsystem mock_gpio_subsystem;
  ASSERT_EQ(mock_gpio_subsystem.stop(), false);
}
#pragma clang diagnostic pop