#include <fcntl.h>
#include <unistd.h>
#include "TimestampManager.h"
#include <string>
#include <sstream>

TimestampManager::TimestampManager() = default;


bool TimestampManager::Poll() {
  int fd_ = open("/sys/kernel/time_stamper/ts_buffer", O_RDWR);

  if (fd_ == -1) {
    return false;
  }

  unsigned char temp_buffer[BUFFER_SIZE];
  ssize_t bytes = read(fd_, &temp_buffer, BUFFER_SIZE);
  if (bytes == 0) {
    return false;

  }
  std::stringstream ss;
  for (int i = 0; i < bytes; i++) {
    ss << temp_buffer[i];
  }

  last_timestamp_ = std::stod(ss.str());
  close(fd_);

  return true;
}
double TimestampManager::GetLastTimestamp() const {
  return last_timestamp_;
}

TimestampManager::~TimestampManager() = default;