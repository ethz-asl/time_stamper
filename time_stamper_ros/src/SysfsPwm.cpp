#include <SysfsPwm.h>
#include <unistd.h>
#include <cstring>
#include <dirent.h>

SysfsPwm::SysfsPwm(std::string pwmchip_path)
    : pwm_chip_path_(std::move(pwmchip_path)) {}

bool SysfsPwm::IsExported() {
  std::string pathstr = pwm_chip_path_ + PWM0;
  if (DirectoryExists(pathstr.c_str())) {
    return true;
  }
  return false;
}

bool SysfsPwm::Reset() {
  bool isReset = Unexport()
      && Export()
      && Write(PWM_PERIOD, std::to_string(PWM_DEFAULT_PERIOD))
      && Write(PWM_DUTYCYCLE, std::to_string(PWM_DEFAULT_DUTYCYCLE));

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
  return Write(PWM_EXPORT, "0");
}

bool SysfsPwm::Unexport() {
  return Write(PWM_UNEXPORT, "0");
}

bool SysfsPwm::IsRunning() {
  int a = 0;
  bool has_read = Read(PWM_ENABLE, &a, 1);

  if (!has_read) {
    return false;
  }

  return a == 1;
}

bool SysfsPwm::Start() {
  return Write(PWM_ENABLE, "1");
}
bool SysfsPwm::Stop() {
  return Write(PWM_ENABLE, "0");
}

bool SysfsPwm::SetFrequency(int hz) {

  if (hz == 0) {
    return false;
  }

  int freq = (int) 1e9 / hz;
  bool r = ChangeDutyCycleRaw(freq / 2);
  if (!r) {
    return false;
  }
  return Write(PWM_PERIOD, std::to_string(freq));
}

bool SysfsPwm::ChangeDutyCycle(int percentage) {
  std::string value_str = std::to_string(5000000);
  return Write(PWM_DUTYCYCLE, value_str);
}

bool SysfsPwm::ChangeDutyCycleRaw(int value) {
  std::string value_str = std::to_string(value);
  return Write(PWM_DUTYCYCLE, value_str);
}

bool SysfsPwm::Write(const std::string &path, const std::string &message) {
  std::string pathstr = pwm_chip_path_ + path;
  int fd = open(pathstr.c_str(), O_WRONLY);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == message.size();
}

bool SysfsPwm::Read(const std::string &path, void *buffer, size_t buffer_size) {
  std::string pathstr = pwm_chip_path_ + path;
  int fd = open(pathstr.c_str(), O_RDONLY);
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
