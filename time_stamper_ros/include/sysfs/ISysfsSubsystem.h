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
  virtual bool IsExported() = 0;

  /**
   * Exports subsystem.
   * @return true if successful, otherwise false.
   */
  virtual bool Export() = 0;

  /**
   * Contrary to Export()
   * @return true if successful, otherwise false.
   */
  virtual bool Unexport() = 0;

  /**
   * Checks if subsystem is enabled/running
   * @return true if running, otherwise false.
   */
  virtual bool IsRunning() = 0;

  /**
   * Enables subsystem, IsRunning() is set to true. Not every subsystem supports this function.
   * @return true if successful, otherwise false.
   */
  virtual bool Start() = 0;

  /**
   * Disables subsystem, IsRunning() is set to false. Not every subsystem supports this function.
   * @return true if successful, otherwise false.
   */
  virtual bool Stop() = 0;

  /**
   * Default destructor
   */
  virtual ~ISysfsSubsystem() = default;

 protected:
  inline static const char* SYSFS_PATH = "/sys/class";
  inline static const char* SYSFS_EXPORT = "/export";
  inline static const char* SYSFS_UNEXPORT = "/unexport";
};

