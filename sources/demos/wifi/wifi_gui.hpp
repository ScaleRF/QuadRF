#pragma once

#include "wifi_constants.hpp"
#include "wifi_dsp.hpp"

#include <SDL2/SDL.h>

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace quadrf_wifi {

struct GuiTelemetry {
  std::string rate_name = "6 Mbps (BPSK 1/2)";

  bool tunnel_up = true;
  std::string tun_name = "tun0";
  std::string ip_cidr = "10.99.0.1/24";
  std::string mac_str = "02:00:0a:63:00:01";

  uint64_t tx_packets = 0;
  uint64_t rx_frames = 0;
  uint64_t fcs_ok = 0;
  uint64_t fcs_fail = 0;

  float last_snr_db = 0.f;
  float last_evm_db = 0.f;
  float last_cfo_hz = 0.f;
  float last_energy = 0.f;
  double throughput_kbps = 0.0;

  std::string ping_target = "10.99.0.2";
  std::string ping_status = "Idle - Click [PING PEER] to test link";
  float ping_rtt_ms = 0.f;
  float ping_min_rtt_ms = 0.f;
  float ping_max_rtt_ms = 0.f;
  float ping_avg_rtt_ms = 0.f;
  uint32_t ping_sent = 0;
  uint32_t ping_rcvd = 0;
  bool ping_in_progress = false;
  bool auto_ping = false;
};

struct GuiButton {
  SDL_Rect rect;
  std::string label;
  std::function<void()> onClick;
  SDL_Color bg;
  SDL_Color border;
  SDL_Color text_color;
};

class WifiGui {
 public:
  WifiGui(int width = 880, int height = 436);
  ~WifiGui();

  bool init();
  void close();

  void add_constellation_points(const std::vector<c32>& points);
  void update_telemetry(const GuiTelemetry& telem);
  bool render_frame();

  void on_toggle_tunnel(std::function<void()> cb) { cb_toggle_tunnel_ = cb; }
  void on_change_rate(std::function<void(int dir)> cb) { cb_rate_ = cb; }
  void on_switch_preset(std::function<void()> cb) { cb_switch_preset_ = cb; }
  void on_send_ping(std::function<void()> cb) { cb_send_ping_ = cb; }
  void on_toggle_auto_ping(std::function<void()> cb) { cb_toggle_auto_ping_ = cb; }

 private:
  int width_;
  int height_;
  bool initialized_ = false;

  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;

  std::mutex telem_mu_;
  GuiTelemetry telem_;

  std::mutex points_mu_;
  std::deque<c32> display_points_;
  static constexpr size_t kMaxDisplayPoints = 1200;

  std::vector<GuiButton> buttons_;

  std::function<void()> cb_toggle_tunnel_;
  std::function<void(int)> cb_rate_;
  std::function<void()> cb_switch_preset_;
  std::function<void()> cb_send_ping_;
  std::function<void()> cb_toggle_auto_ping_;

  void draw_string(int x, int y, const std::string& text, SDL_Color color, int scale = 1);
  void draw_button(const GuiButton& btn, bool is_hover = false);
  void draw_card(int x, int y, int w, int h, const std::string& title, SDL_Color border);
  void draw_constellation(int cx, int cy, int radius);
  void draw_spectrum_meter(int x, int y, int w, int h);
  void rebuild_buttons();
  void draw_ui();
};

}  // namespace quadrf_wifi
