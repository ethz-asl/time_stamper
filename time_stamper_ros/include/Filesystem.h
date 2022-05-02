#pragma once

#include <string>
#include "IFileSystem.h"

class Filesystem : public IFileSystem {

  /**
   * Internal write function. Wrapper for posix write(2) with some checks.
   * @param path
   * @param message
   * @return true if successful, otherwise false and errno is set.
   */
  bool write(const std::string &path, const std::string &message) override;

  /**
   * Wrapper for posix read(2) with some checks.
   * @param path Absolute path
   * @param buffer buffer to read data to
   * @param buffer_size sizeof(buffer)
   * @return true if successful, otherwise false and errno is set.
   */
  bool read(const std::string &path, void *buffer, size_t buffer_size) override;

  /**
   * Checks if a directory exists at given location
   * @param path Absolute path
   * @return true if directory exists, otherwise false
   */
  bool directoryExists(const char *path) override;
};

