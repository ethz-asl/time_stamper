#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include "Filesystem.h"

bool Filesystem::Write(const std::string &path, const std::string &message) {
  int fd = open((path).c_str(), O_WRONLY);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == message.size();
}

bool Filesystem::Read(const std::string &path, void *buffer, size_t buffer_size) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  ssize_t nbytes = read(fd, buffer, buffer_size);
  close(fd);
  return nbytes == buffer_size;
}

bool Filesystem::DirectoryExists(const char *path) {
  if (path == nullptr) {
    return false;
  }

  DIR *pDir = opendir(path);

  bool bExists = false;

  if (pDir != nullptr) {
    bExists = true;
    closedir(pDir);
  }

  return bExists;
}
