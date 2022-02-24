#pragma once

#include <fcntl.h>
class IPwmSubsystem {
 public:
  /**
   * Interface default constructor.
   */
  IPwmSubsystem() = default;

  /**
   * Checks if pwmchip is exported.
   * @return if true pwmchip is exported, otherwise false.
   */
  virtual bool IsExported() = 0;

  /**
   * Exports pwmchip.
   * @return true if successful, otherwise false.
   */
  virtual bool Export() = 0;

  /**
   * Contrary to Export()
   * @return true if successful, otherwise false.
   */
  virtual bool Unexport() = 0;

  /**
   * Checks if pwmchip is enabled/running
   * @return true if running, otherwise false.
   */
  virtual bool IsRunning() = 0;

  /**
   * Enables pwmchip, IsRunning() is set to true.
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool Start() = 0;

  /**
   * Disables pwmchip, IsRunning() is set to false.
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool Stop() = 0;

  /**
   * Sets clock frequency.
   * @param hz
   * @return true if set, otherwise false.
   */
  virtual bool SetFrequency(int hz) = 0;

  /**
   * Experimental function. Use at your own risk
   * @param hz
   * @return true if successful, otherwise false.
   */
  virtual bool ChangeDutyCycle(int hz) = 0;

  /**
   * Internal function to set duty cycle
   * @param value raw value
   * @return true if set successful, otherwise false.
   */
  virtual bool ChangeDutyCycleRaw(int value) = 0;

  /**
   * Internal write function. Wrapper for posix write(2) with some checks.
   * @param path
   * @param message
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool Write(const std::string &path, const std::string &message) = 0;

  /**
   * Internal read function. Wrapper for posix read(2) with some checks.
   * @param path relative to pwmchip path
   * @param buffer buffer to read data to
   * @param buffer_size sizeof(buffer)
   * @return true if successful, otherwise false and errno is set.
   */
  virtual bool Read(const std::string &path, void *buffer, size_t buffer_size) = 0;

  /**
   * Useful when pwmchip is in a undefined state.
   * Restarts pwmchip, Runs for a short period with default configuration and stops again.
   * @return true if successful, otherwise false.
   */
  virtual bool Reset() = 0;

  /**
   * Default destructor
   */
  ~IPwmSubsystem() = default;
};
