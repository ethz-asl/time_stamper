#include <fcntl.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <ctime>
#include <unistd.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <iostream>

#define ONE_SEC 1000000000ULL
#define PTP_MAX_DEV_PATH 16

clockid_t fd_to_clockid(const int fd) {
  return ((~fd)) << 3 | 3;
}

void open_phc_fd(int *fd_ptp, const std::string& ifname) {
  struct ethtool_ts_info interface_info = {ETHTOOL_GET_TS_INFO};
  struct ifreq req = {0};

  /* Get PHC index */
  snprintf(req.ifr_name, sizeof(req.ifr_name), "%s", ifname.c_str());

  req.ifr_data = (char *) &interface_info;

  int fd_ioctl = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ioctl < 0) {
    std::cout << "Couldn't open socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (ioctl(fd_ioctl, SIOCETHTOOL, &req) < 0) {
    std::cout << "Couldn't issue SIOCETHTOOL ioctl" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string ptp_path("/dev/ptp");
  ptp_path += std::to_string(interface_info.phc_index);

  if (ptp_path.size() > PTP_MAX_DEV_PATH) {
    return;
  }

  *fd_ptp = open(ptp_path.c_str(), O_RDONLY);
  if (*fd_ptp < 0) {
    std::cout << "Couldn't open the PTP fd. Did you forget to run with sudo again?" << std::endl;
    exit(EXIT_FAILURE);
  }

  close(fd_ioctl);
}

int main() {
  struct timespec ts_ptp1{}, ts_ptp2{};

  std::string interface = "enp0s31f6";
  if (interface.size() > IFNAMSIZ) {
    return -1;
  }

  int fd_ptp;
  open_phc_fd(&fd_ptp, interface);

  /* Fetch timestamps for each clock. */
  clock_gettime(fd_to_clockid(fd_ptp), &ts_ptp1);
  uint64_t ptp = (ts_ptp1.tv_sec * ONE_SEC) + ts_ptp1.tv_nsec;

  /* Compute clocks read latency. */
  clock_gettime(fd_to_clockid(fd_ptp), &ts_ptp1);
  clock_gettime(fd_to_clockid(fd_ptp), &ts_ptp2);
  uint64_t lat_ptp = ((ts_ptp2.tv_sec * ONE_SEC) + ts_ptp2.tv_nsec) - ((ts_ptp1.tv_sec * ONE_SEC) + ts_ptp1.tv_nsec);

  std::cout << "phc timestamp: " << ptp << std::endl;
  std::cout << "phc latency: " << lat_ptp << std::endl;

  close(fd_ptp);

  return 0;
}