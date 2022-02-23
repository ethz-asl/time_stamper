#pragma once
#include <string>
#include <fcntl.h>
#include "IPwmSubsystem.h"

class SysfsPwm : public IPwmSubsystem {
 public:
  /**
   * SysfsPwm constructor.
   * @param pwmchip_path path to pwmchip e.g. /sys/class/pwm/pwmchip0
   */
  explicit SysfsPwm(std::string pwmchip_path);

  /**
   * Checks if pwmchip is exported.
   * @return if true pwmchip is exported, otherwise false.
   */
  bool IsExported() override;

  /**
   * Exports pwmchip.
   * @return true if successful, otherwise false.
   */
  bool Export() override;

  /**
   * Contrary to Export()
   * @return true if successful, otherwise false.
   */
  bool Unexport() override;

  /**
   * Checks if pwmchip is enabled/running
   * @return true if running, otherwise false.
   */
  bool IsRunning() override;

  /**
   * Enables pwmchip, IsRunning() is set to true.
   * @return true if successful, otherwise false and errno is set.
   */
  bool Start() override;

  /**
   * Disables pwmchip, IsRunning() is set to false.
   * @return true if successful, otherwise false and errno is set.
   */

  bool Stop() override;

  /**
   * Sets clock frequency.
   * @param hz
   * @return true if set, otherwise false.
   */
  bool SetFrequency(int hz) override;

  /**
   * Useful when pwmchip is in a undefined state.
   * Restarts pwmchip, Runs for a short period with default configuration and stops again.
   * @return true if successful, otherwise false.
   */
  bool Reset() override;

  [[deprecated("This function is experimental and might break.")]]
  bool ChangeDutyCycle(int value) override;

  /**
   * Default destructor.
   */
  ~SysfsPwm() = default;

 private:
  /**
   * Internal write function. Wrapper for posix write(2) with some checks.
   * @param path
   * @param message
   * @return true if successful, otherwise false and errno is set.
   */
  bool Write(const std::string &path, const std::string &message) override;

  /**
   * Internal read function. Wrapper for posix read(2) with some checks.
   * @param path relative to pwmchip path
   * @param buffer buffer to read data to
   * @param buffer_size sizeof(buffer)
   * @return true if successful, otherwise false and errno is set.
   */
  bool Read(const std::string &path, void *buffer, size_t buffer_size) override;

  /**
   * Stores pwmchip path.
   */
  std::string pwm_chip_path_;

  /**
   * Internal utility function to check if directory exists
   * @param path Absolute path to directory
   * @return true if directory exists, otherwise false.
   */
  static bool DirectoryExists(const char *path);

  /**
   * Internal function to set duty cycle
   * @param value raw value
   * @return true if set successful, otherwise false.
   */
  bool ChangeDutyCycleRaw(int value) override;
};
