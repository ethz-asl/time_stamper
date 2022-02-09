#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "IPwmSubsystem.h"
#include "Node.h"
class MockPwmSubsystem : public IPwmSubsystem {

 public:
  MOCK_METHOD0(IsExported, bool());
  MOCK_METHOD0(Export, bool());
  MOCK_METHOD0(Unexport, bool());
  MOCK_METHOD0(IsRunning, bool());
  MOCK_METHOD0(Start, bool());
  MOCK_METHOD0(Stop, bool());
  MOCK_METHOD1(SetFrequency, bool(int hz));
  MOCK_METHOD1(ChangeDutyCycle, bool(int hz));
  MOCK_METHOD1(ChangeDutyCycleRaw, bool(int value));
  MOCK_METHOD2(Write, bool(const std::string& path, const std::string& message));
  MOCK_METHOD3(Read, bool(const std::string& path, void* buffer, size_t buffer_size));
};

TEST(MockPwmSubsystem, TestIsExported) {
  MockPwmSubsystem mock_pwm_subsystem;
  Node node(mock_pwm_subsystem);
}