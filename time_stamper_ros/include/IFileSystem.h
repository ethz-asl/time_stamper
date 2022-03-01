#pragma once

class IFileSystem {
 public:
  /**
   * Interface default constructor.
   */
  IFileSystem() = default;

  virtual bool Write(const std::string &path, const std::string &message) = 0;
  virtual bool Read(const std::string &path, void *buffer, size_t buffer_size) = 0;
  virtual bool DirectoryExists(const char *path) = 0;

  /**
   * Default destructor
   */
  ~IFileSystem() = default;
};
