#include <SysfsPwm.h>
#include <unistd.h>
#include <cstring>

bool SysfsPwm::IsExported() {
  return false;
}

bool SysfsPwm::_Export() {
  return sysfsctl("/sys/class/pwm/pwmchip0/export", "0", O_WRONLY);
}

bool SysfsPwm::Unexport() {
  return sysfsctl("/sys/class/pwm/pwmchip0/unexport", "0", O_WRONLY);
}

bool SysfsPwm::IsRunning() {
  int a = 0;
  bool has_read = sysfsread("/sys/class/pwm/pwmchip0/pwm0/enable", &a, 1);

  //TODO improve exception handling
  if (!has_read) {
    return false;
  }
  return a;
}

bool SysfsPwm::Start() {
  return sysfsctl("/sys/class/pwm/pwmchip0/pwm0/enable", "1");
}
bool SysfsPwm::Stop() {
  return sysfsctl("/sys/class/pwm/pwmchip0/pwm0/enable", "0");
}

bool SysfsPwm::SetFrequency(int hz) {
  int a = 10000000;
  bool r = ChangeDutyCycleRaw(a / 2);
  if (!r) {
    return false;
  }
  return sysfsctl("/sys/class/pwm/pwmchip0/pwm0/period", std::to_string(a));
}

bool SysfsPwm::ChangeDutyCycle(int percentage) {
  std::string value_str = std::to_string(5000000);
  return sysfsctl("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", value_str, O_WRONLY);
}

bool SysfsPwm::ChangeDutyCycleRaw(int value) {
  std::string value_str = std::to_string(value);
  return sysfsctl("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", value_str, O_WRONLY);
}

bool SysfsPwm::sysfsctl(const std::string& path, const std::string& message, int file_flags) {
  int fd = open(path.c_str(), file_flags);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == message.size();
}

bool SysfsPwm::sysfsread(const std::string& path, void* buffer, size_t buffer_size, int file_flags) {
  int fd = open(path.c_str(), file_flags);
  if (fd == -1) {
    return false;
  }
  ssize_t nbytes = read(fd, buffer, buffer_size);
  return nbytes == buffer_size;
}
