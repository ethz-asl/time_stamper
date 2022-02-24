#pragma once
#include <string>
#include <fcntl.h>
#include "IPwmSubsystem.h"

#define SYSFS_EXPERIMENTAL [[deprecated("This function is experimental and might break.")]]

/**
 * Interface functions are documented in IPwmSubsystem.h
 */
class SysfsPwm : public IPwmSubsystem {
 public:
  /**
   * SysfsPwm constructor.
   * @param pwmchip_path path to pwmchip e.g. /sys/class/pwm/pwmchip0
   */
  explicit SysfsPwm(std::string pwmchip_path);

  /**
   * Interface functions are documented in IPwmSubsystem.h
   */
  bool IsExported() override;
  bool Export() override;
  bool Unexport() override;
  bool IsRunning() override;
  bool Start() override;
  bool Stop() override;
  bool SetFrequency(int hz) override;
  bool Reset() override;
  SYSFS_EXPERIMENTAL bool ChangeDutyCycle(int percentage) override;

  /**
   * Default destructor.
   */
  ~SysfsPwm() = default;

 private:
  /**
   * IPwmSubsystem internal functions
   */
  bool Write(const std::string &path, const std::string &message) override;
  bool Read(const std::string &path, void *buffer, size_t buffer_size) override;
  static bool DirectoryExists(const char *path);
  bool ChangeDutyCycleRaw(int value) override;

  /**
   * Stores pwmchip path.
   */
  std::string pwm_chip_path_;

  /**
   * Internal constants
   */
  static constexpr int PWM_DEFAULT_PERIOD = 10000000;
  static constexpr int PWM_DEFAULT_DUTYCYCLE = PWM_DEFAULT_PERIOD / 2;

  inline static const std::string PWM0 = "/pwm0";
  inline static const std::string PWM_EXPORT = "/export";
  inline static const std::string PWM_UNEXPORT = "/unexport";
  inline static const std::string PWM_ENABLE = "/pwm0/enable";
  inline static const std::string PWM_PERIOD = "/pwm0/period";
  inline static const std::string PWM_DUTYCYCLE = "/pwm0/duty_cycle";
};
