#pragma once
#include <string>
#include <fcntl.h>
#include "sysfs/IPwmSubsystem.h"
#include "IFileSystem.h"

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
  SysfsPwm(std::string pwmchip_path, IFileSystem &file_system);

  /**
   * Interface functions are documented in IPwmSubsystem.h
   */
  bool isExported() override;
  bool exprt() override;
  bool unexport() override;
  bool isRunning() override;
  bool start() override;
  bool stop() override;
  bool setFrequency(int hz) override;
  bool reset() override;
  bool getFrequency(void *buffer, ssize_t size) override;
  SYSFS_EXPERIMENTAL bool changeDutyCycle(int percentage) override;

  /**
   * Default destructor.
   */
  ~SysfsPwm() override = default;

 private:
  /**
   * IPwmSubsystem internal functions
   */
  bool changeDutyCycleRaw(int value) override;

  /**
   * Stores pwmchip path.
   */
  std::string pwm_chip_path_;

  /**
   * Interface to access filesystem
   */

  IFileSystem &fs_;
};
