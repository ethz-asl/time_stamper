#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "sysfs/SysfsPwm.h"
#include "../Mocks/MockFilesystem.h"

using ::testing::Return;

TEST(MockFileSystem, TestSetFrequencyInvalidInput) {
  //Precondition: Subsystem is in any valid state
  MockFilesystem mock_filesystem;
  SysfsPwm sysfs_pwm("/sys/class/pwm/pwmchip0", mock_filesystem);


  //Expectation: Subsystem should filter invalid values

  bool setFrequencyResult = sysfs_pwm.SetFrequency(IPwmSubsystem::PWM_MAXSPEED_HZ + 1);
  EXPECT_EQ(setFrequencyResult, false);
  setFrequencyResult = sysfs_pwm.SetFrequency(0);
  EXPECT_EQ(setFrequencyResult, false);
  setFrequencyResult = sysfs_pwm.SetFrequency(-1);
  EXPECT_EQ(setFrequencyResult, false);
}

TEST(MockFileSystem, TestSetFrequencyValidInputBasic) {
  //Precondition: Subsystem is in any valid state
  MockFilesystem mock_filesystem;
  SysfsPwm sysfs_pwm("/sys/class/pwm/pwmchip0", mock_filesystem);

  //Expectation: Subsystem should set dutycycle and period on valid input
  int hz = 1;
  int freq = (int) (1e9 / hz);
  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_DUTYCYCLE, "0"))
  .Times(1).WillOnce(Return(true));

  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_PERIOD, std::to_string(freq)))
  .Times(1).WillOnce(Return(true));

  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_DUTYCYCLE, std::to_string(freq / 2)))
  .Times(1).WillOnce(Return(true));

  bool setFrequencyResult = sysfs_pwm.SetFrequency(hz);
  EXPECT_EQ(setFrequencyResult, true);
}

TEST(MockFileSystem, TestSetFrequencyValidInputLimit) {
  //Precondition: Subsystem is in any valid state
  MockFilesystem mock_filesystem;
  SysfsPwm sysfs_pwm("/sys/class/pwm/pwmchip0", mock_filesystem);

  //Expectation: Subsystem should set dutycycle and period on valid input
  int hz = IPwmSubsystem::PWM_MAXSPEED_HZ;
  int freq = (int) (1e9 / hz);
  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_DUTYCYCLE, "0"))
  .Times(1).WillOnce(Return(true));

  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_PERIOD, std::to_string(freq)))
  .Times(1).WillOnce(Return(true));

  EXPECT_CALL(mock_filesystem, write("/sys/class/pwm/pwmchip0" + SysfsPwm::PWM_DUTYCYCLE, std::to_string(freq / 2)))
  .Times(1).WillOnce(Return(true));

  bool setFrequencyResult = sysfs_pwm.SetFrequency(hz);
  EXPECT_EQ(setFrequencyResult, true);

}