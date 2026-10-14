#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace quadrf_wifi {

struct ClientMsg {
  std::string type;
  std::string key;
  double val = 0.0;
  std::string str_val;
  bool bool_val = false;
};

using MsgCallback = std::function<void(const ClientMsg&)>;

class WifiServer {
 public:
  WifiServer(int port = 8092, std::string web_dir = "");
  ~WifiServer();

  WifiServer(const WifiServer&) = delete;
  WifiServer& operator=(const WifiServer&) = delete;

  bool start(std::string& err_msg);
  void stop();
  void poll(int timeout_ms = 10);

  void set_msg_callback(MsgCallback cb) { msg_cb_ = std::move(cb); }

  void broadcast_json(const std::string& json_str);

  int port() const { return port_; }
  const std::string& web_dir() const { return web_dir_; }

 private:
  int port_ = 8092;
  std::string web_dir_;
  MsgCallback msg_cb_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace quadrf_wifi
