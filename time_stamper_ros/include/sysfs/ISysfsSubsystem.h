#pragma once

#define SYSFS_NOT_SUPPORTED [[deprecated("This function is not supported")]]

class ISysfsSubsystem {
 public:

  /**
   * Interface default constructor.
   */
  ISysfsSubsystem() = default;

  /**
   * Checks if subsystem is exported.
   * @return if true pwmchip is exported, otherwise false.
   */
  virtual bool isExported() = 0;

  /**
   * Typo is intended to avoid C++ keyword.
   * Exports subsystem.
   * @return true if successful, otherwise false.
   */
  virtual bool exprt() = 0;

  /**
   * Contrary to exprt()
   * @return true if successful, otherwise false.
   */
  virtual bool unexport() = 0;

  /**
   * Checks if subsystem is enabled/running
   * @return true if running, otherwise false.
   */
  virtual bool isRunning() = 0;

  /**
   * Enables subsystem, isRunning() is set to true. Not every subsystem supports this function.
   * @return true if successful, otherwise false.
   */
  virtual bool start() = 0;

  /**
   * Disables subsystem, isRunning() is set to false. Not every subsystem supports this function.
   * @return true if successful, otherwise false.
   */
  virtual bool stop() = 0;

  /**
   * Default destructor
   */
  virtual ~ISysfsSubsystem() = default;

 protected:
  inline static const char* SYSFS_PATH = "/sys/class";
  inline static const char* SYSFS_EXPORT = "/export";
  inline static const char* SYSFS_UNEXPORT = "/unexport";
};

