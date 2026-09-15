#include "wifi_gui.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>

namespace quadrf_wifi {

namespace {

// Minimal embedded 5x7 bitmap font for ASCII 32..126
static const uint8_t kFont5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42 '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64 '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 'V'
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // 87 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91 '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93 ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96 '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 103 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 106 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 112 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 113 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 121 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123 '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124 '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125 '}'
    {0x08, 0x08, 0x2A, 0x1C, 0x08}  // 126 '~'
};

}  // namespace

WifiGui::WifiGui(int width, int height) : width_(width), height_(height) {}

WifiGui::~WifiGui() {
  close();
}

bool WifiGui::init() {
  if (initialized_) return true;

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::cerr << "WifiGui: SDL_Init failed: " << SDL_GetError() << "\n";
    return false;
  }

  window_ = SDL_CreateWindow("802.11 Data Link",
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             width_, height_, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (!window_) {
    std::cerr << "WifiGui: SDL_CreateWindow failed: " << SDL_GetError() << "\n";
    return false;
  }

  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer_) {
    renderer_ = SDL_CreateRenderer(window_, -1, 0);
  }

  initialized_ = (renderer_ != nullptr);
  return initialized_;
}

void WifiGui::close() {
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  if (initialized_) {
    SDL_Quit();
    initialized_ = false;
  }
}

void WifiGui::draw_string(int x, int y, const std::string& text, SDL_Color color, int scale) {
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  int cur_x = x;
  for (char ch : text) {
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t* glyph = kFont5x7[ch - 32];
    for (int col = 0; col < 5; ++col) {
      uint8_t line = glyph[col];
      for (int row = 0; row < 7; ++row) {
        if (line & (1 << row)) {
          if (scale == 1) {
            SDL_RenderDrawPoint(renderer_, cur_x + col, y + row);
          } else {
            SDL_Rect pixel{cur_x + col * scale, y + row * scale, scale, scale};
            SDL_RenderFillRect(renderer_, &pixel);
          }
        }
      }
    }
    cur_x += (5 + 1) * scale;
  }
}

void WifiGui::draw_card(int x, int y, int w, int h, const std::string& title, SDL_Color border) {
  SDL_Rect fill{x, y, w, h};
  SDL_SetRenderDrawColor(renderer_, 22, 30, 46, 255);
  SDL_RenderFillRect(renderer_, &fill);

  SDL_SetRenderDrawColor(renderer_, border.r, border.g, border.b, border.a);
  SDL_RenderDrawRect(renderer_, &fill);

  if (!title.empty()) {
    draw_string(x + 12, y + 8, title, border);
    SDL_SetRenderDrawColor(renderer_, border.r / 2, border.g / 2, border.b / 2, 255);
    SDL_RenderDrawLine(renderer_, x + 1, y + 22, x + w - 2, y + 22);
  }
}

void WifiGui::draw_button(const GuiButton& btn, bool /*is_hover*/) {
  SDL_SetRenderDrawColor(renderer_, btn.bg.r, btn.bg.g, btn.bg.b, btn.bg.a);
  SDL_RenderFillRect(renderer_, &btn.rect);

  SDL_SetRenderDrawColor(renderer_, btn.border.r, btn.border.g, btn.border.b, btn.border.a);
  SDL_RenderDrawRect(renderer_, &btn.rect);

  int text_w = static_cast<int>(btn.label.size()) * 6;
  int text_x = btn.rect.x + std::max(4, (btn.rect.w - text_w) / 2);
  int text_y = btn.rect.y + (btn.rect.h - 7) / 2;
  draw_string(text_x, text_y, btn.label, btn.text_color);
}

void WifiGui::add_constellation_points(const std::vector<c32>& points) {
  std::lock_guard<std::mutex> lock(points_mu_);
  for (const auto& pt : points) {
    if (display_points_.size() >= kMaxDisplayPoints) {
      display_points_.pop_front();
    }
    display_points_.push_back(pt);
  }
}

void WifiGui::update_telemetry(const GuiTelemetry& telem) {
  std::lock_guard<std::mutex> lock(telem_mu_);
  telem_ = telem;
}

void WifiGui::draw_constellation(int cx, int cy, int radius) {
  SDL_Rect box{cx - radius - 10, cy - radius - 10, (radius + 10) * 2, (radius + 10) * 2};
  SDL_SetRenderDrawColor(renderer_, 15, 20, 30, 255);
  SDL_RenderFillRect(renderer_, &box);
  SDL_SetRenderDrawColor(renderer_, 56, 189, 248, 120);
  SDL_RenderDrawRect(renderer_, &box);

  SDL_SetRenderDrawColor(renderer_, 51, 65, 85, 255);
  SDL_RenderDrawLine(renderer_, cx - radius, cy, cx + radius, cy);
  SDL_RenderDrawLine(renderer_, cx, cy - radius, cx, cy + radius);

  for (int a = 0; a < 360; a += 6) {
    float rad = static_cast<float>(a) * kPi / 180.f;
    int px = cx + static_cast<int>(std::cos(rad) * radius * 0.707f);
    int py = cy + static_cast<int>(std::sin(rad) * radius * 0.707f);
    SDL_RenderDrawPoint(renderer_, px, py);
  }

  std::lock_guard<std::mutex> lock(points_mu_);
  SDL_SetRenderDrawColor(renderer_, 34, 197, 94, 230);
  const float scale = static_cast<float>(radius) * 0.75f;
  for (const auto& pt : display_points_) {
    int px = cx + static_cast<int>(pt.real() * scale);
    int py = cy - static_cast<int>(pt.imag() * scale);
    if (px >= box.x + 2 && px <= box.x + box.w - 3 && py >= box.y + 2 && py <= box.y + box.h - 3) {
      SDL_RenderDrawPoint(renderer_, px, py);
      SDL_RenderDrawPoint(renderer_, px + 1, py);
      SDL_RenderDrawPoint(renderer_, px, py + 1);
    }
  }

  draw_string(cx - radius + 4, cy - radius - 4, "IQ", {56, 189, 248, 255});
}

void WifiGui::draw_spectrum_meter(int x, int y, int w, int h) {
  SDL_Rect box{x, y, w, h};
  SDL_SetRenderDrawColor(renderer_, 15, 20, 30, 255);
  SDL_RenderFillRect(renderer_, &box);
  SDL_SetRenderDrawColor(renderer_, 71, 85, 105, 255);
  SDL_RenderDrawRect(renderer_, &box);

  GuiTelemetry t;
  {
    std::lock_guard<std::mutex> lock(telem_mu_);
    t = telem_;
  }

  float en = std::min(1.0f, t.last_energy * 12.f);
  int fill_w = static_cast<int>(static_cast<float>(w - 4) * en);
  SDL_Rect bar{x + 2, y + 2, fill_w, h - 4};
  
  if (t.last_snr_db > 15.f) {
    SDL_SetRenderDrawColor(renderer_, 34, 197, 94, 255);
  } else if (t.last_snr_db > 8.f) {
    SDL_SetRenderDrawColor(renderer_, 56, 189, 248, 255);
  } else {
    SDL_SetRenderDrawColor(renderer_, 245, 158, 11, 255);
  }
  SDL_RenderFillRect(renderer_, &bar);

  char buf[128];
  std::snprintf(buf, sizeof(buf),
                "ADC Sample Power: %.4f | LTF SNR: %.1f dB | EVM: %.1f dB | CFO: %+.0f Hz",
                t.last_energy, t.last_snr_db, t.last_evm_db, t.last_cfo_hz);
  draw_string(x + 10, y + (h - 7) / 2, buf, {241, 245, 249, 255});
}

void WifiGui::rebuild_buttons() {
  buttons_.clear();

  GuiTelemetry t;
  {
    std::lock_guard<std::mutex> lock(telem_mu_);
    t = telem_;
  }

  GuiButton btn_tun;
  btn_tun.rect = {30, 126, 190, 32};
  if (t.tunnel_up) {
    btn_tun.label = "[ STOP TUNNEL ]";
    btn_tun.bg = {185, 28, 28, 255};
    btn_tun.border = {239, 68, 68, 255};
    btn_tun.text_color = {255, 255, 255, 255};
  } else {
    btn_tun.label = "[ START TUNNEL ]";
    btn_tun.bg = {21, 128, 61, 255};
    btn_tun.border = {34, 197, 94, 255};
    btn_tun.text_color = {255, 255, 255, 255};
  }
  btn_tun.onClick = [this]() {
    if (cb_toggle_tunnel_) cb_toggle_tunnel_();
  };
  buttons_.push_back(btn_tun);

  GuiButton btn_preset;
  btn_preset.rect = {230, 126, 180, 32};
  bool is_node1 = (t.ip_cidr.find("10.99.0.1") != std::string::npos);
  btn_preset.label = is_node1 ? "Role: Node 1 (0.1)" : "Role: Node 2 (0.2)";
  btn_preset.bg = {30, 41, 59, 255};
  btn_preset.border = {56, 189, 248, 255};
  btn_preset.text_color = {56, 189, 248, 255};
  btn_preset.onClick = [this]() {
    if (cb_switch_preset_) cb_switch_preset_();
  };
  buttons_.push_back(btn_preset);

  GuiButton btn_rate_prev;
  btn_rate_prev.rect = {30, 196, 90, 28};
  btn_rate_prev.label = "< Rate";
  btn_rate_prev.bg = {30, 41, 59, 255};
  btn_rate_prev.border = {71, 85, 105, 255};
  btn_rate_prev.text_color = {241, 245, 249, 255};
  btn_rate_prev.onClick = [this]() {
    if (cb_rate_) cb_rate_(-1);
  };
  buttons_.push_back(btn_rate_prev);

  GuiButton btn_rate_next;
  btn_rate_next.rect = {130, 196, 90, 28};
  btn_rate_next.label = "Rate >";
  btn_rate_next.bg = {30, 41, 59, 255};
  btn_rate_next.border = {71, 85, 105, 255};
  btn_rate_next.text_color = {241, 245, 249, 255};
  btn_rate_next.onClick = [this]() {
    if (cb_rate_) cb_rate_(1);
  };
  buttons_.push_back(btn_rate_next);

  GuiButton btn_ping;
  btn_ping.rect = {30, 244, 185, 28};
  btn_ping.label = "[ PING PEER ]";
  btn_ping.bg = {2, 132, 199, 255};
  btn_ping.border = {56, 189, 248, 255};
  btn_ping.text_color = {255, 255, 255, 255};
  btn_ping.onClick = [this]() {
    if (cb_send_ping_) cb_send_ping_();
  };
  buttons_.push_back(btn_ping);

  GuiButton btn_auto_ping;
  btn_auto_ping.rect = {225, 244, 185, 28};
  btn_auto_ping.label = t.auto_ping ? "[ AUTO-PING: ON ]" : "[ AUTO-PING: OFF ]";
  btn_auto_ping.bg = t.auto_ping ? SDL_Color{21, 128, 61, 255} : SDL_Color{30, 41, 59, 255};
  btn_auto_ping.border = t.auto_ping ? SDL_Color{34, 197, 94, 255} : SDL_Color{71, 85, 105, 255};
  btn_auto_ping.text_color = {255, 255, 255, 255};
  btn_auto_ping.onClick = [this]() {
    if (cb_toggle_auto_ping_) cb_toggle_auto_ping_();
  };
  buttons_.push_back(btn_auto_ping);
}

void WifiGui::draw_ui() {
  GuiTelemetry t;
  {
    std::lock_guard<std::mutex> lock(telem_mu_);
    t = telem_;
  }

  char stat_buf[64];
  if (t.tunnel_up) {
    std::snprintf(stat_buf, sizeof(stat_buf), "[TUNNEL ACTIVE - %s]", t.ip_cidr.c_str());
    draw_string(20, 16, stat_buf, {34, 197, 94, 255});
  } else {
    std::snprintf(stat_buf, sizeof(stat_buf), "[TUNNEL INACTIVE]");
    draw_string(20, 16, stat_buf, {239, 68, 68, 255});
  }

  draw_spectrum_meter(20, 38, width_ - 40, 26);

  draw_card(20, 74, 410, 246, "", {56, 189, 248, 255});

  char buf[128];
  std::snprintf(buf, sizeof(buf), "TUN Device : %s (%s)", t.tun_name.c_str(), t.ip_cidr.c_str());
  draw_string(32, 86, buf, {241, 245, 249, 255});

  std::snprintf(buf, sizeof(buf), "Node MAC   : %s", t.mac_str.c_str());
  draw_string(32, 102, buf, {148, 163, 184, 255});

  std::snprintf(buf, sizeof(buf), "Modulation : %s", t.rate_name.c_str());
  draw_string(32, 176, buf, {245, 158, 11, 255});

  for (const auto& btn : buttons_) {
    draw_button(btn);
  }

  draw_string(32, 280, t.ping_status, {56, 189, 248, 255});

  draw_constellation(650, 197, 113);

  draw_card(20, 332, width_ - 40, 92, "LINK TELEMETRY & LIVE PING METRICS", {34, 197, 94, 255});

  const uint64_t total_rx = t.fcs_ok + t.fcs_fail;
  const float pass_rate = total_rx > 0 ? (100.f * static_cast<float>(t.fcs_ok) / static_cast<float>(total_rx)) : 0.f;

  std::snprintf(buf, sizeof(buf), "TX: %llu pkts | RX: %llu frames | FCS OK: %llu (%.1f%%) | Tput: %.1f kbps",
                static_cast<unsigned long long>(t.tx_packets),
                static_cast<unsigned long long>(t.rx_frames),
                static_cast<unsigned long long>(t.fcs_ok),
                pass_rate,
                t.throughput_kbps);
  draw_string(32, 366, buf, {241, 245, 249, 255});

  const float loss_rate = (t.ping_sent > 0) ? (100.f * (1.f - static_cast<float>(t.ping_rcvd) / static_cast<float>(t.ping_sent))) : 0.f;
  std::snprintf(buf, sizeof(buf), "Ping Peer (%s): %u sent, %u rcvd (%.0f%% loss) | RTT: %.1f ms (min %.1f, avg %.1f, max %.1f)",
                t.ping_target.c_str(), t.ping_sent, t.ping_rcvd, loss_rate,
                t.ping_rtt_ms, t.ping_min_rtt_ms, t.ping_avg_rtt_ms, t.ping_max_rtt_ms);
  draw_string(32, 386, buf, (t.ping_rcvd > 0) ? SDL_Color{34, 197, 94, 255} : SDL_Color{245, 158, 11, 255});

  draw_string(32, 406, "Shortcuts: [Return/I] Ping Now | [A] Auto-Ping | [Space/U] Tunnel | [R] Rate | [Q/Esc] Quit",
              {148, 163, 184, 255});
}

bool WifiGui::render_frame() {
  if (!initialized_) return false;

  rebuild_buttons();

  SDL_Event ev;
  while (SDL_PollEvent(&ev)) {
    if (ev.type == SDL_QUIT) {
      return false;
    } else if (ev.type == SDL_MOUSEBUTTONDOWN) {
      if (ev.button.button == SDL_BUTTON_LEFT) {
        int mx = ev.button.x;
        int my = ev.button.y;
        for (const auto& btn : buttons_) {
          if (mx >= btn.rect.x && mx <= btn.rect.x + btn.rect.w &&
              my >= btn.rect.y && my <= btn.rect.y + btn.rect.h) {
            if (btn.onClick) {
              btn.onClick();
            }
            break;
          }
        }
      }
    } else if (ev.type == SDL_KEYDOWN) {
      switch (ev.key.keysym.sym) {
        case SDLK_ESCAPE:
        case SDLK_q:
          return false;
        case SDLK_SPACE:
        case SDLK_u:
          if (cb_toggle_tunnel_) cb_toggle_tunnel_();
          break;
        case SDLK_p:
          if (cb_switch_preset_) cb_switch_preset_();
          break;
        case SDLK_RETURN:
        case SDLK_i:
          if (cb_send_ping_) cb_send_ping_();
          break;
        case SDLK_a:
          if (cb_toggle_auto_ping_) cb_toggle_auto_ping_();
          break;
        case SDLK_r:
          if (cb_rate_) cb_rate_(1);
          break;
        default:
          break;
      }
    }
  }

  SDL_SetRenderDrawColor(renderer_, 11, 15, 23, 255);
  SDL_RenderClear(renderer_);
  draw_ui();

  SDL_RenderPresent(renderer_);
  return true;
}

}  // namespace quadrf_wifi
