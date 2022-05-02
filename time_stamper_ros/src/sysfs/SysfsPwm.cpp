#include "sysfs/SysfsPwm.h"
#include <unistd.h>

SysfsPwm::SysfsPwm(std::string pwmchip_path, IFileSystem &file_system)
    : pwm_chip_path_(std::move(pwmchip_path)), fs_(file_system) {}

bool SysfsPwm::IsExported() {
  return fs_.directoryExists((pwm_chip_path_ + PWM0).c_str());
}

bool SysfsPwm::Reset() {
  bool isReset = Unexport()
      && Export()
      && fs_.write(pwm_chip_path_ + PWM_PERIOD, std::to_string(PWM_DEFAULT_PERIOD))
      && fs_.write(pwm_chip_path_ + PWM_DUTYCYCLE, std::to_string(PWM_DEFAULT_DUTYCYCLE));

  if (!isReset) {
    return false;
  }

  if (!Start()) {
    return false;
  }
  //Run pwmchip for 10ms before stopping
  usleep(1e3 * 10);
  Stop();

  //Give pwmchip 10ms time to stop
  usleep(1e3 * 10);
  return true;
}

bool SysfsPwm::Export() {
  return fs_.write(pwm_chip_path_ + SYSFS_EXPORT, "0");
}

bool SysfsPwm::Unexport() {
  return fs_.write(pwm_chip_path_ + SYSFS_UNEXPORT, "0");
}

bool SysfsPwm::IsRunning() {
  int a = 0;
  bool has_read = fs_.read(pwm_chip_path_ + PWM_ENABLE, &a, 1);

  if (!has_read) {
    return false;
  }

  return a == 1;
}

bool SysfsPwm::Start() {
  return fs_.write(pwm_chip_path_ + PWM_ENABLE, "1");
}

bool SysfsPwm::Stop() {
  return fs_.write(pwm_chip_path_ + PWM_ENABLE, "0");
}

bool SysfsPwm::SetFrequency(int hz) {

  if (hz > PWM_MAXSPEED_HZ || hz <= 0) {
    errno = EINVAL;
    return false;
  }

  /* Dutycycle needs to be smaller than period or else pwm0 throws invalid argument error.
  Set to 0 to ignore previous state and avoid errors */
  fs_.write(pwm_chip_path_ + PWM_DUTYCYCLE, "0");

  int freq = (int) 1e9 / hz;
  bool r = fs_.write(pwm_chip_path_ + PWM_PERIOD, std::to_string(freq));
  if (!r) {
    return false;
  }
  return ChangeDutyCycleRaw(freq / 2); //50% DutyCycle
}

bool SysfsPwm::ChangeDutyCycle(int percentage) {
  if (percentage > 100 || percentage <= 0) {
    errno = EINVAL;
    return false;
  }

  int freq = 0;
  if (!GetFrequency(&freq, sizeof(freq))) {
    return false;
  }
  int raw_duty_cycle = (freq / 100) * percentage;
  return fs_.write(pwm_chip_path_ + PWM_DUTYCYCLE, std::to_string(raw_duty_cycle));
}

bool SysfsPwm::ChangeDutyCycleRaw(int value) {
  std::string value_str = std::to_string(value);
  return fs_.write(pwm_chip_path_ + PWM_DUTYCYCLE, value_str);
}

bool SysfsPwm::GetFrequency(void *buffer, ssize_t size) {
  return fs_.read(pwm_chip_path_ + PWM_PERIOD, buffer, size);
}
