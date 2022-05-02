#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "MockPwmSubsystem.h"
#include "MockGpioSubsystem.h"
#include "Node.h"

using ::testing::Return;

/**
 * Helper function to expect default GpioSubsystem calls.
 * @param mock_gpio_subsystem
 */
void ExpectDefaultGpioSubsystem(MockGpioSubsystem &mock_gpio_subsystem) {
  EXPECT_CALL(mock_gpio_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, SetDirection(OUT)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_gpio_subsystem, SetGpioMode(HIGH)).Times(1).WillOnce(Return(true));
}

TEST(MockPwmSubsystem, TestNodeInitWhenUninitialized) {
  //Precondition: Pwm Subsystem is not enabled and not exported.
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  //Expectation: Export, set frequency to 50 and start subsystem.
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Export()).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Start()).WillOnce(Return(true));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitUninitializedWithForceReset) {
  //Precondition: Subsystem is not enabled and not exported
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  //Expectation: Export, set frequency to 50 and start subsystem.
  EXPECT_CALL(mock_pwm_subsystem, Reset()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Export()).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Start()).WillOnce(Return(true));

  node.Init(50, true);
}

TEST(MockPwmSubsystem, TestNodeInitFailExport) {
  //Precondition: Subsystem is not enabled and not exported
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  //Expectation: Subsystem fails export.
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Export()).WillOnce(Return(false));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitAlreadyExported) {
  //Precondition: Subsystem is not running but already exported
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  //Expectation: Subsystem does not export, sets frequency to 50 and starts.
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Start()).WillOnce(Return(true));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitAlreadyRunning) {
  //Precondition: Subsystem is exported and already running
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  // Subsystem sets frequency to 50
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(true));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitFailSetFrequency) {
  //Precondition: Subsystem is not running but exported
  MockPwmSubsystem mock_pwm_subsystem;
  MockGpioSubsystem mock_gpio_subsystem;
  Node node(mock_pwm_subsystem, mock_gpio_subsystem, FPS);

  ExpectDefaultGpioSubsystem(mock_gpio_subsystem);
  //Expectation: Subsystem fails to set frequency
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(false));

  node.Init(50, false);
}
