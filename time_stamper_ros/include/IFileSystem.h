#pragma once

class IFileSystem {
 public:
  /**
   * Interface default constructor.
   */
  IFileSystem() = default;

  virtual bool write(const std::string &path, const std::string &message) = 0;
  virtual bool read(const std::string &path, void *buffer, size_t buffer_size) = 0;
  virtual bool directoryExists(const char *path) = 0;

  /**
   * Default destructor
   */
  ~IFileSystem() = default;
};
