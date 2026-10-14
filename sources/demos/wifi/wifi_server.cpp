#include "wifi_server.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

#if __has_include("../mongoose.h")
#include "../mongoose.h"
#elif __has_include("mongoose.h")
#include "mongoose.h"
#else
#include "../../demos/mongoose.h"
#endif

namespace quadrf_wifi {

struct WifiServer::Impl {
  WifiServer* parent = nullptr;
  struct mg_mgr mgr;
  std::mutex mu;
  bool running = false;

  static void ev_handler(struct mg_connection* c, int ev, void* ev_data) {
    auto* self = static_cast<Impl*>(c->fn_data);
    if (!self) return;

    if (ev == MG_EV_HTTP_MSG) {
      auto* hm = static_cast<struct mg_http_message*>(ev_data);
      if (mg_match(hm->uri, mg_str("/ws"), NULL) || mg_match(hm->uri, mg_str("/data/ws"), NULL)) {
        mg_ws_upgrade(c, hm, NULL);
      } else if (mg_match(hm->uri, mg_str("/api/apps/attach"), NULL) ||
                 mg_match(hm->uri, mg_str("/api/apps/detach"), NULL)) {
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"status\":\"ok\"}\n");
      } else {
        struct mg_http_serve_opts opts = {};
        opts.root_dir = self->parent->web_dir().c_str();
        opts.extra_headers = "Cache-Control: no-store\r\n";
        mg_http_serve_dir(c, hm, &opts);
      }
    } else if (ev == MG_EV_WS_MSG) {
      auto* wm = static_cast<struct mg_ws_message*>(ev_data);
      self->handle_ws_msg(c, wm->data);
    }
  }

  void handle_ws_msg([[maybe_unused]] struct mg_connection* c, struct mg_str msg) {
    std::string s(msg.buf, msg.len);
    ClientMsg cm;

    // Simple JSON parser for supported commands
    auto get_str = [&](const std::string& key) -> std::string {
      std::string q = "\"" + key + "\"";
      size_t pos = s.find(q);
      if (pos == std::string::npos) return "";
      pos += q.size();
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
      if (pos >= s.size() || s[pos] != ':') return "";
      ++pos;
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
      if (pos >= s.size() || s[pos] != '"') return "";
      ++pos;
      size_t end = s.find("\"", pos);
      if (end == std::string::npos) return "";
      return s.substr(pos, end - pos);
    };

    auto get_num = [&](const std::string& key) -> double {
      std::string q = "\"" + key + "\":";
      size_t pos = s.find(q);
      if (pos == std::string::npos) return 0.0;
      pos += q.size();
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
      if (pos < s.size() && s[pos] == '"') {
        ++pos;
        size_t end = s.find("\"", pos);
        if (end == std::string::npos) return 0.0;
        try { return std::stod(s.substr(pos, end - pos)); } catch (...) { return 0.0; }
      }
      char* end_p = nullptr;
      double v = std::strtod(&s[pos], &end_p);
      return v;
    };

    auto get_bool = [&](const std::string& key) -> bool {
      std::string q = "\"" + key + "\":";
      size_t pos = s.find(q);
      if (pos == std::string::npos) return false;
      pos += q.size();
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
      if (s.compare(pos, 4, "true") == 0) return true;
      if (s.compare(pos, 1, "1") == 0) return true;
      return false;
    };

    cm.type = get_str("t");
    if (cm.type.empty()) {
      if (s.find("\"ping\"") != std::string::npos) cm.type = "ping";
      else if (s.find("\"hello\"") != std::string::npos) cm.type = "hello";
      else return;
    }

    if (cm.type == "set") {
      cm.key = get_str("k");
      cm.val = get_num("v");
      cm.str_val = get_str("v");
      cm.bool_val = get_bool("v");
    } else if (cm.type == "chat") {
      cm.str_val = get_str("text");
    } else if (cm.type == "auto_ping") {
      cm.bool_val = get_bool("v");
    } else if (cm.type == "viz") {
      cm.bool_val = get_bool("on");
    }

    if (parent->msg_cb_) {
      parent->msg_cb_(cm);
    }
  }

  void broadcast(const std::string& data) {
    std::lock_guard<std::mutex> lock(mu);
    if (!running) return;
    for (struct mg_connection* c = mgr.conns; c != NULL; c = c->next) {
      if (c->is_websocket) {
        mg_ws_send(c, data.data(), data.size(), WEBSOCKET_OP_TEXT);
      }
    }
  }
};

WifiServer::WifiServer(int port, std::string web_dir)
    : port_(port), web_dir_(std::move(web_dir)), impl_(std::make_unique<Impl>()) {
  impl_->parent = this;
}

WifiServer::~WifiServer() {
  stop();
}

bool WifiServer::start(std::string& err_msg) {
  stop();
  mg_mgr_init(&impl_->mgr);
  mg_log_set(MG_LL_ERROR);

  char url[64];
  std::snprintf(url, sizeof(url), "http://0.0.0.0:%d", port_);
  struct mg_connection* c = mg_http_listen(&impl_->mgr, url, Impl::ev_handler, impl_.get());
  if (!c) {
    err_msg = "Failed to bind HTTP/WebSocket listener on " + std::string(url);
    return false;
  }

  impl_->running = true;
  std::cerr << "WifiServer: listening on " << url << " (web: " << web_dir_ << ")\n";
  return true;
}

void WifiServer::stop() {
  if (impl_->running) {
    impl_->running = false;
    mg_mgr_free(&impl_->mgr);
  }
}

void WifiServer::poll(int timeout_ms) {
  if (impl_->running) {
    mg_mgr_poll(&impl_->mgr, timeout_ms);
  }
}

void WifiServer::broadcast_json(const std::string& json_str) {
  impl_->broadcast(json_str);
}

}  // namespace quadrf_wifi
