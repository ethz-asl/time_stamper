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
  bool isExported() override = 0;

  /**
   * Exports pwmchip.
   * @return true if successful, otherwise false.
   */
  bool exprt() override = 0;

  /**
   * Contrary to exprt()
   * @return true if successful, otherwise false.
   */
  bool unexport() override = 0;

  /**
   * Checks if pwmchip is enabled/running
   * @return true if running, otherwise false.
   */
  bool isRunning() override = 0;

  /**
   * Enables pwmchip, isRunning() is set to true.
   * @return true if successful, otherwise false and errno is set.
   */
  bool start() override = 0;

  /**
   * Disables pwmchip, isRunning() is set to false.
   * @return true if successful, otherwise false and errno is set.
   */
  bool stop() override = 0;

  /**
   * Gets clock frequency in hz
   * @return
   */
  virtual bool getFrequency(void *buffer, ssize_t size) = 0;

  /**
   * Sets clock frequency.
   * @param hz
   * @return true if set, otherwise false.
   */
  virtual bool setFrequency(int hz) = 0;

  /**
   * This method should only be used if a dutycycle other than 50 is required.
   * Sets the Dutycycle percentage. Experimental function, use at your own risk.
   * @param percentage Dutycycle percentage (1-100).
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool changeDutyCycle(int percentage) = 0;

  /**
   * Internal function to set duty cycle
   * @param value raw value
   * @return true if set successful, otherwise false.
   */
  virtual bool changeDutyCycleRaw(int value) = 0;

  /**
   * Useful when pwmchip is in a undefined state.
   * Restarts pwmchip, Runs for a short period with default configuration and stops again.
   * @return true if successful, otherwise false.
   */
  virtual bool reset() = 0;

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
