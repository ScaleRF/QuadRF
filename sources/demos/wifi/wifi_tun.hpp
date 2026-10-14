#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace quadrf_wifi {

template <typename T>
class SafeQueue {
 public:
  void push(T item) {
    {
      std::lock_guard<std::mutex> lock(mu_);
      if constexpr (std::is_same_v<T, std::vector<uint8_t>>) {
        uint32_t new_flow = 0;
        bool new_pure = false;
        if (is_ipv4_tcp_ack(item, new_flow, new_pure)) {
          for (auto it = queue_.begin(); it != queue_.end();) {
            uint32_t old_flow = 0;
            bool old_pure = false;
            if (is_ipv4_tcp_ack(*it, old_flow, old_pure) && old_pure && old_flow == new_flow) {
              it = queue_.erase(it);
            } else {
              ++it;
            }
          }
        }
      }
      queue_.push_back(std::move(item));
    }
    cv_.notify_one();
  }

  bool pop(T& item, int timeout_ms = 100) {
    std::unique_lock<std::mutex> lock(mu_);
    if (cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                     [&] { return !queue_.empty() || stop_; })) {
      if (queue_.empty()) return false;
      item = std::move(queue_.front());
      queue_.pop_front();
      return true;
    }
    return false;
  }

  bool try_pop(T& item) {
    std::lock_guard<std::mutex> lock(mu_);
    if (queue_.empty()) return false;
    item = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  bool try_pop_if_fits(T& item, size_t max_bytes) {
    std::lock_guard<std::mutex> lock(mu_);
    if (queue_.empty() || queue_.front().size() > max_bytes) return false;
    item = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return queue_.size();
  }

  void stop() {
    stop_ = true;
    cv_.notify_all();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mu_);
    queue_.clear();
  }

 private:
  static bool is_ipv4_tcp_ack(const std::vector<uint8_t>& pkt, uint32_t& flow_hash, bool& is_pure_ack) {
    if (pkt.size() < 40 || (pkt[0] >> 4) != 4 || pkt[9] != 6) return false;
    const size_t ihl = static_cast<size_t>(pkt[0] & 0x0Fu) * 4;
    const size_t tot_len = (static_cast<size_t>(pkt[2]) << 8) | pkt[3];
    if (ihl < 20 || tot_len > pkt.size() || tot_len < ihl + 20) return false;
    const uint8_t* tcp = pkt.data() + ihl;
    const size_t thl = static_cast<size_t>((tcp[12] >> 4) & 0x0Fu) * 4;
    if (thl < 20 || tot_len < ihl + thl) return false;
    const uint8_t flags = tcp[13];
    if ((flags & 0x17u) != 0x10u) return false;
    is_pure_ack = (tot_len == ihl + thl) && (thl <= 32);
    uint32_t h = 2166136261u;
    for (size_t i = 12; i < 20; ++i) h = (h ^ pkt[i]) * 16777619u;
    for (size_t i = 0; i < 4; ++i) h = (h ^ tcp[i]) * 16777619u;
    flow_hash = h;
    return true;
  }

  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<T> queue_;
  bool stop_ = false;
};

class WifiTun {
 public:
  WifiTun();
  ~WifiTun();

  bool open(const std::string& dev_name = "tun0");
  bool configure_interface(const std::string& ip_cidr, int mtu = 1400);
  void start();
  void stop();
  bool inject_packet(const uint8_t* data, size_t len);
  SafeQueue<std::vector<uint8_t>>& tx_queue() { return tx_queue_; }

  const std::string& ifname() const { return ifname_; }
  bool is_open() const { return fd_ >= 0; }

 private:
  int fd_ = -1;
  std::string ifname_;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  SafeQueue<std::vector<uint8_t>> tx_queue_;

  void read_loop();
};

}  // namespace quadrf_wifi
