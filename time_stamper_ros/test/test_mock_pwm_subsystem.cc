#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "IPwmSubsystem.h"
#include "Node.h"

using ::testing::Return;

//TODO Update Unittests - fix Reset() warnings
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
  MOCK_METHOD2(Write, bool(const std::string& path, const std::string& message));
  MOCK_METHOD3(Read, bool(const std::string& path, void* buffer, size_t buffer_size));
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
  EXPECT_CALL(mock_pwm_subsystem, Start).WillOnce(Return(true));

  node.Init();
}

TEST(MockPwmSubsystem, TestNodeInitFailExport) {
  //Precondition: Subsystem is not enabled and not exported
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);

  //Expectation: Subsystem fails export.
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(false));
  EXPECT_CALL(mock_pwm_subsystem, Export()).WillOnce(Return(false));

  node.Init();
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

  node.Init();
}

TEST(MockPwmSubsystem, TestNodeInitAlreadyRunning) {
  //Precondition: Subsystem is exported and already running
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);

  // Subsystem sets frequency to 50
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, IsRunning()).WillOnce(Return(true));

  node.Init();
}

TEST(MockPwmSubsystem, TestNodeInitFailSetFrequency) {
  //Precondition: Subsystem is not running but exported
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);

  //Expectation: Subsystem fails to set frequency
  EXPECT_CALL(mock_pwm_subsystem, IsExported()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(false));

  node.Init();
}

TEST(MockPwmSubsystem, TestFrequencyValueValidation) {
  //Precondition: Subsystem is in any valid state
  MockPwmSubsystem mock_pwm_subsystem;
  //TODO define valid range

  //Expectation: Subsystem should filter invalid values
  EXPECT_CALL(mock_pwm_subsystem, SetFrequency(50)).WillOnce(Return(true));
  mock_pwm_subsystem.SetFrequency(50);
}

TEST(MockPwmSubsystem, TestNodeInitSanityCheck) {
  //Precondition: Subsystem is not exported but running, which is undefined behavior and should stop node instantly

 //TODO implement
}


