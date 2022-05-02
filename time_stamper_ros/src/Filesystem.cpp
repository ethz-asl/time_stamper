#include <dirent.h>
#include "sys/stat.h"
#include <unistd.h>
#include <fcntl.h>
#include "Filesystem.h"

bool Filesystem::write(const std::string &path, const std::string &message) {
  int fd = open((path).c_str(), O_WRONLY);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = ::write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == message.size();
}

bool Filesystem::read(const std::string &path, void *buffer, size_t buffer_size) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  ssize_t nbytes = ::read(fd, buffer, buffer_size);
  close(fd);
  return nbytes == buffer_size;
}

bool Filesystem::directoryExists(const char *path) {
  if (path == nullptr) {
    return false;
  }

  struct stat fileInfo{};
  if (lstat(path, &fileInfo) != 0) {
    return false;
  }

  return S_ISDIR(fileInfo.st_mode);
}
