#include "wifi_tun.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/route.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace quadrf_wifi {

WifiTun::WifiTun() = default;

WifiTun::~WifiTun() {
  stop();
}

bool WifiTun::open(const std::string& dev_name) {
  stop();

  fd_ = ::open("/dev/net/tun", O_RDWR);
  if (fd_ < 0) {
    std::cerr << "WifiTun: failed to open /dev/net/tun: " << std::strerror(errno) << "\n";
    return false;
  }

  struct ifreq ifr{};
  ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
  if (!dev_name.empty()) {
    std::strncpy(ifr.ifr_name, dev_name.c_str(), IFNAMSIZ - 1);
  }

  if (ioctl(fd_, TUNSETIFF, reinterpret_cast<void*>(&ifr)) < 0) {
    std::cerr << "WifiTun: ioctl(TUNSETIFF) failed: " << std::strerror(errno) << "\n";
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  ifname_ = ifr.ifr_name;
  std::cerr << "WifiTun: opened interface " << ifname_ << " (fd=" << fd_ << ")\n";
  return true;
}

bool WifiTun::configure_interface(const std::string& ip_cidr, int mtu) {
  if (fd_ < 0 || ifname_.empty()) return false;

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "WifiTun: socket(AF_INET) failed: " << std::strerror(errno) << "\n";
    return false;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);

  // 1. Set MTU
  ifr.ifr_mtu = mtu;
  if (::ioctl(sock, SIOCSIFMTU, &ifr) < 0) {
    std::cerr << "WifiTun: SIOCSIFMTU failed: " << std::strerror(errno) << "\n";
  }

  // 2. Assign IP and netmask if provided
  if (!ip_cidr.empty()) {
    std::string ip_str = ip_cidr;
    int prefix = 24;
    auto slash = ip_cidr.find('/');
    if (slash != std::string::npos) {
      ip_str = ip_cidr.substr(0, slash);
      try {
        prefix = std::stoi(ip_cidr.substr(slash + 1));
      } catch (...) {
        prefix = 24;
      }
    }

    struct sockaddr_in sin{};
    sin.sin_family = AF_INET;
    if (::inet_pton(AF_INET, ip_str.c_str(), &sin.sin_addr) == 1) {
      std::memcpy(&ifr.ifr_addr, &sin, sizeof(sin));
      if (::ioctl(sock, SIOCSIFADDR, &ifr) < 0) {
        std::cerr << "WifiTun: SIOCSIFADDR failed: " << std::strerror(errno) << "\n";
      }

      uint32_t mask = (prefix <= 0) ? 0 : (prefix >= 32) ? 0xFFFFFFFFU : (~0U << (32 - prefix));
      sin.sin_addr.s_addr = htonl(mask);
      std::memcpy(&ifr.ifr_netmask, &sin, sizeof(sin));
      if (::ioctl(sock, SIOCSIFNETMASK, &ifr) < 0) {
        std::cerr << "WifiTun: SIOCSIFNETMASK failed: " << std::strerror(errno) << "\n";
      }
      // 3. Bring interface UP
      if (::ioctl(sock, SIOCGIFFLAGS, &ifr) >= 0) {
        ifr.ifr_flags |= (IFF_UP | IFF_RUNNING);
        if (::ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
          std::cerr << "WifiTun: SIOCSIFFLAGS(IFF_UP) failed: " << std::strerror(errno) << "\n";
        }
      }

      // 4. Add subnet route
      struct rtentry rt{};
      auto* sin_dst = reinterpret_cast<struct sockaddr_in*>(&rt.rt_dst);
      auto* sin_mask = reinterpret_cast<struct sockaddr_in*>(&rt.rt_genmask);

      sin_dst->sin_family = AF_INET;
      sin_dst->sin_addr.s_addr = sin.sin_addr.s_addr & htonl(mask);

      sin_mask->sin_family = AF_INET;
      sin_mask->sin_addr.s_addr = htonl(mask);

      rt.rt_dev = const_cast<char*>(ifname_.c_str());
      rt.rt_flags = RTF_UP;

      if (::ioctl(sock, SIOCADDRT, &rt) < 0 && errno != EEXIST) {
        // Fallback: non-fatal if route already exists
      }
    }
  } else {
    // Bring up without IP
    if (::ioctl(sock, SIOCGIFFLAGS, &ifr) >= 0) {
      ifr.ifr_flags |= (IFF_UP | IFF_RUNNING);
      if (::ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        std::cerr << "WifiTun: SIOCSIFFLAGS(IFF_UP) failed: " << std::strerror(errno) << "\n";
      }
    }
  }

  ::close(sock);

  // Disable IPv6 autoconf/multicast on SDR TUN to save channel airtime
  char cmd[256];
  std::snprintf(cmd, sizeof(cmd), "sysctl -w net.ipv6.conf.%s.disable_ipv6=1 >/dev/null 2>&1 || true", ifname_.c_str());
  std::system(cmd);

  std::cerr << "WifiTun: configured " << ifname_ << " (" << ip_cidr << ", MTU " << mtu << ")\n";
  return true;
}

void WifiTun::start() {
  if (running_ || fd_ < 0) return;
  running_ = true;
  reader_thread_ = std::thread(&WifiTun::read_loop, this);
}

void WifiTun::stop() {
  if (running_) {
    running_ = false;
    tx_queue_.stop();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool WifiTun::inject_packet(const uint8_t* data, size_t len) {
  if (fd_ < 0 || !data || len == 0) return false;
  ssize_t written = ::write(fd_, data, len);
  return written == static_cast<ssize_t>(len);
}

void WifiTun::read_loop() {
  uint8_t buf[2048];
  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  while (running_) {
    int ret = poll(&pfd, 1, 50);  // 50 ms timeout
    if (ret > 0 && (pfd.revents & POLLIN)) {
      ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n > 0) {
        std::vector<uint8_t> pkt(buf, buf + n);
        tx_queue_.push(std::move(pkt));
      } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
        break;
      }
    }
  }
}

}  // namespace quadrf_wifi
