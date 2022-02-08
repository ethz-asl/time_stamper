#include "Node.h"
#include <unistd.h>
#include <cstring>
#include "fcntl.h"
Node::Node() {

}

bool Node::Init() {
  /*
    //SysfsPwm sysfs_pwm1(1);
    int fd = open("/sys/kernel/time_stamper/ts_buffer", O_RDWR);
    if (fd == -1) {
      std::cout << "Error %i from open: " << errno << " " << strerror(errno) << std::endl;
      return false;
    }


      int buffer_size = 4096;
      while (true) {
        unsigned char temp_buffer[buffer_size];
        ssize_t n = read(fd, &temp_buffer, buffer_size);
        if (n > 0) {
          std::cout << "Read bytes: " << n << std::endl;
          for (int i = 0; i < n; i++) {
            std::cout << temp_buffer[i];
          }
          std::cout << std::endl;
        }
      }

  // echo 10000000 > pwm0/period
  // echo 5000000 > pwm0/duty_cycle
  // echo 1 > pwm0/enable

  std::string path = "/sys/class/pwm/pwmchip0/export";
  fd = open(path.c_str(), O_WRONLY);
  if (fd == -1) {
    printf("Error %i from open: %s\n", errno, strerror(errno));

    return false;
  }

  if (write(fd, "0", 1) != 1) {
    std::cout << "Error writing to " << path << std::endl;
  }

  close(fd);

  fd = open("/sys/class/pwm/pwmchip0/pwm0/period", O_WRONLY);
  write(fd, "10000000", 8);
  close(fd);

  fd = open("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", O_WRONLY);
  write(fd, "5000000", 7);
  close(fd);

  fd = open("/sys/class/pwm/pwmchip0/pwm0/enable", O_WRONLY);
  write(fd, "1", 1);
  */
  return true;
}
