#pragma once

#include <fcntl.h>
#include "ISysfsSubsystem.h"
class IPwmSubsystem : public ISysfsSubsystem {
 public:
  /**
   * Interface default constructor.
   */
  IPwmSubsystem() = default;

  /**
   * Checks if pwmchip is exported.
   * @return if true pwmchip is exported, otherwise false.
   */
  bool IsExported() override = 0;

  /**
   * Exports pwmchip.
   * @return true if successful, otherwise false.
   */
  bool Export() override = 0;

  /**
   * Contrary to Export()
   * @return true if successful, otherwise false.
   */
  bool Unexport() override = 0;

  /**
   * Checks if pwmchip is enabled/running
   * @return true if running, otherwise false.
   */
  bool IsRunning() override = 0;

  /**
   * Enables pwmchip, IsRunning() is set to true.
   * @return true if successful, otherwise false and errno is set.
   */
  bool Start() override = 0;

  /**
   * Disables pwmchip, IsRunning() is set to false.
   * @return true if successful, otherwise false and errno is set.
   */
  bool Stop() override = 0;

  /**
   * Gets clock frequency in hz
   * @return
   */
  virtual bool GetFrequency(void *buffer, ssize_t size) = 0;

  /**
   * Sets clock frequency.
   * @param hz
   * @return true if set, otherwise false.
   */
  virtual bool SetFrequency(int hz) = 0;

  /**
   * This method should only be used if a dutycycle other than 50 is required.
   * Sets the Dutycycle percentage. Experimental function, use at your own risk.
   * @param percentage Dutycycle percentage (1-100).
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool ChangeDutyCycle(int percentage) = 0;

  /**
   * Internal function to set duty cycle
   * @param value raw value
   * @return true if set successful, otherwise false.
   */
  virtual bool ChangeDutyCycleRaw(int value) = 0;

  /**
   * Useful when pwmchip is in a undefined state.
   * Restarts pwmchip, Runs for a short period with default configuration and stops again.
   * @return true if successful, otherwise false.
   */
  virtual bool Reset() = 0;

  /**
   * Internal constants
   */
  static constexpr int PWM_MAXSPEED_HZ = 10526315; //95ns
  static constexpr int PWM_DEFAULT_PERIOD = 10000000;
  static constexpr int PWM_DEFAULT_DUTYCYCLE = PWM_DEFAULT_PERIOD / 2;

  inline static const std::string PWM0 = "/pwm0";
  inline static const std::string PWM_ENABLE = "/pwm0/enable";
  inline static const std::string PWM_PERIOD = "/pwm0/period";
  inline static const std::string PWM_DUTYCYCLE = "/pwm0/duty_cycle";

  /**
   * Default destructor
   */
  ~IPwmSubsystem() override = default;
};
