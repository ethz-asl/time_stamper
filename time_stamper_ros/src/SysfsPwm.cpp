#include <SysfsPwm.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include "fcntl.h"

SysfsPwm::SysfsPwm(int test)
    : test_(test) {

}

bool SysfsPwm::isExported() {
  return false;
}

bool SysfsPwm::_export() {
  return false;
}
bool SysfsPwm::unexport() {
  return false;
}
bool SysfsPwm::isRunning() {
  int fd = open("/sys/class/pwm/pwmchip0/pwm0/enable", O_RDONLY);
  if (fd == -1) {
    return false;
  }

  char a;
  read(fd, &a, 1);
  std::cout << a << std::endl;
  return true;
}

bool SysfsPwm::start() {

  return false;
}
bool SysfsPwm::stop() {
  return false;
}
bool SysfsPwm::changeFrequency() {
  return false;
}
bool SysfsPwm::changeDutyCycle(int hz) {
  return false;
}
bool SysfsPwm::changeDutyCycleRaw(int value) {
  return false;
}
