#include <SysfsPwm.h>
#include <unistd.h>
#include <cstring>
#include <dirent.h>

bool SysfsPwm::IsExported() {
  if (DirectoryExists("/sys/class/pwm/pwmchip0/pwm0")) {
    return true;
  }
  return false;
}

bool SysfsPwm::Reset() {
  bool isReset = Unexport()
  && Export()
  && Write("/sys/class/pwm/pwmchip0/pwm0/period", "10000000")
  && Write("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", "5000000");

  if (!isReset) {
    return false;
  }

  if (!Start()) {
    return false;
  }
  usleep(1e3 * 10);
  Stop();
  return true;
}

bool SysfsPwm::Export() {
  return Write("/sys/class/pwm/pwmchip0/export", "0");
}

bool SysfsPwm::Unexport() {
  return Write("/sys/class/pwm/pwmchip0/unexport", "0");
}

bool SysfsPwm::IsRunning() {
  int a = 0;
  bool has_read = Read("/sys/class/pwm/pwmchip0/pwm0/enable", &a, 1);

  if (!has_read) {
    return false;
  }

  if (a == 1) {
    return true;
  }
  return false;
}

bool SysfsPwm::Start() {
  return Write("/sys/class/pwm/pwmchip0/pwm0/enable", "1");
}
bool SysfsPwm::Stop() {
  return Write("/sys/class/pwm/pwmchip0/pwm0/enable", "0");
}

bool SysfsPwm::SetFrequency(int hz) {
  int freq = (int) 1e9 / hz;
  bool r = ChangeDutyCycleRaw(freq / 2);
  if (!r) {
    return false;
  }
  return Write("/sys/class/pwm/pwmchip0/pwm0/period", std::to_string(freq));
}

bool SysfsPwm::ChangeDutyCycle(int percentage) {
  std::string value_str = std::to_string(5000000);
  return Write("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", value_str);
}

bool SysfsPwm::ChangeDutyCycleRaw(int value) {
  std::string value_str = std::to_string(value);
  return Write("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", value_str);
}

bool SysfsPwm::Write(const std::string &path, const std::string &message) {
  int fd = open(path.c_str(), O_WRONLY);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == message.size();
}

bool SysfsPwm::Read(const std::string &path, void *buffer, size_t buffer_size) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  ssize_t nbytes = read(fd, buffer, buffer_size);
  return nbytes == buffer_size;
}
bool SysfsPwm::DirectoryExists(const char *path) {
  if (path == nullptr) {
    return false;
  }

  DIR *pDir = opendir(path);

  bool bExists = false;

  if (pDir != nullptr) {
    bExists = true;
    (void) closedir(pDir);
  }

  return bExists;
}
