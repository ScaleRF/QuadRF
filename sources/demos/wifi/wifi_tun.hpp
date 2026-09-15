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
