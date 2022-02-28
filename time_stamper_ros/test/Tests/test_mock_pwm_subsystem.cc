#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "IPwmSubsystem.h"
#include "Node.h"

using ::testing::Return;

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

TEST(MockPwmSubsystem, TestNodeInitUninitialized) {
  //Precondition: Subsystem is not enabled and not exported
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);


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
  Node node(mock_pwm_subsystem);


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
  Node node(mock_pwm_subsystem);

  //Expectation: Subsystem fails export.
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Export()).WillOnce(Return(false));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitAlreadyExported) {
  //Precondition: Subsystem is not running but already exported
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);

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
  Node node(mock_pwm_subsystem);

  // Subsystem sets frequency to 50
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(true));

  node.Init(50, false);
}

TEST(MockPwmSubsystem, TestNodeInitFailSetFrequency) {
  //Precondition: Subsystem is not running but exported
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);

  //Expectation: Subsystem fails to set frequency
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(false));

  node.Init(50, false);
}