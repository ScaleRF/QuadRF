#include "wifi_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace quadrf_wifi {

namespace {

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

WifiLink::WifiLink() : WifiLink(Config{}) {}

WifiLink::WifiLink(Config cfg) : cfg_(cfg) {
  hostname_ = detect_hostname();
  if (cfg_.ip_cidr.empty()) {
    cfg_.ip_cidr = derive_ip_cidr(hostname_);
  }

  // Derive my IP from CIDR
  size_t slash = cfg_.ip_cidr.find('/');
  my_ip_ = (slash != std::string::npos) ? cfg_.ip_cidr.substr(0, slash) : cfg_.ip_cidr;

  // Auto-set peer IP: if I am .2, peer is .3; if I am .3, peer is .2; else .2
  if (my_ip_ == "10.99.0.2") peer_ip_ = "10.99.0.3";
  else if (my_ip_ == "10.99.0.3") peer_ip_ = "10.99.0.2";
  else peer_ip_ = "10.99.0.2";

  // Node MAC: 02:00:0a:63:00:node
  unsigned int ip_b[4]{10, 99, 0, 1};
  std::sscanf(my_ip_.c_str(), "%u.%u.%u.%u", &ip_b[0], &ip_b[1], &ip_b[2], &ip_b[3]);
  my_mac_[0] = 0x02;
  my_mac_[1] = 0x00;
  my_mac_[2] = static_cast<uint8_t>(ip_b[0]);
  my_mac_[3] = static_cast<uint8_t>(ip_b[1]);
  my_mac_[4] = static_cast<uint8_t>(ip_b[2]);
  my_mac_[5] = static_cast<uint8_t>(ip_b[3]);

  // Peer MAC
  unsigned int p_b[4]{10, 99, 0, 2};
  std::sscanf(peer_ip_.c_str(), "%u.%u.%u.%u", &p_b[0], &p_b[1], &p_b[2], &p_b[3]);
  peer_mac_[0] = 0x02;
  peer_mac_[1] = 0x00;
  peer_mac_[2] = static_cast<uint8_t>(p_b[0]);
  peer_mac_[3] = static_cast<uint8_t>(p_b[1]);
  peer_mac_[4] = static_cast<uint8_t>(p_b[2]);
  peer_mac_[5] = static_cast<uint8_t>(p_b[3]);

  const RateInfo* r = rate_info_by_name(cfg_.rate_str);
  if (!r) r = &kRate6;
  current_rate_.store(r);

  if (cfg_.web_dir.empty()) {
#ifdef WIFI_WEB_DIR
    cfg_.web_dir = WIFI_WEB_DIR;
#else
    cfg_.web_dir = "/usr/share/quadrf-data/web";
#endif
  }
}

WifiLink::~WifiLink() {
  stop();
}

std::string WifiLink::detect_hostname() {
  std::string host = "quadrf";
  std::ifstream in("/etc/quadrf/quadrf.conf");
  std::string line;
  while (in && std::getline(in, line)) {
    auto hash = line.find('#');
    if (hash != std::string::npos) line.erase(hash);
    auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    line.erase(0, first);
    const std::string key = "QUADRF_HOSTNAME=";
    if (line.compare(0, key.size(), key) == 0) {
      host = line.substr(key.size());
      auto last = host.find_last_not_of(" \t\r\n");
      if (last != std::string::npos) host.erase(last + 1);
      if (host.size() >= 2 &&
          ((host.front() == '"' && host.back() == '"') ||
           (host.front() == '\'' && host.back() == '\''))) {
        host = host.substr(1, host.size() - 2);
      }
      break;
    }
  }
  if (host == "quadrf") {
    char hnbuf[128] = {};
    if (gethostname(hnbuf, sizeof(hnbuf) - 1) == 0 && hnbuf[0] != '\0') {
      host = hnbuf;
    }
  }
  return host;
}

std::string WifiLink::derive_ip_cidr(const std::string& host) {
  int node = 1;
  const auto dash = host.rfind('-');
  if (dash != std::string::npos) {
    try { node = std::stoi(host.substr(dash + 1)); } catch (...) { node = 1; }
    if (node < 1 || node > 254) node = 1;
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "10.99.0.%d/24", node);
  return buf;
}

bool WifiLink::start(std::string& err_msg) {
  stop();

  // 1. Open TUN interface
  tun_ = std::make_unique<WifiTun>();
  if (!tun_->open(cfg_.tun_name)) {
    err_msg = "Failed to open TUN interface " + cfg_.tun_name;
    return false;
  }
  tun_->configure_interface(cfg_.ip_cidr, cfg_.mtu);
  tun_->start();
  tun_active_.store(true);

  // 2. Initialize Radio
  RadioConfig r_cfg;
  r_cfg.freq_mhz = cfg_.freq_mhz;
  r_cfg.rx_gain = cfg_.rx_gain;
  r_cfg.tx_gain = cfg_.tx_gain;
  r_cfg.tx_antenna_mask = cfg_.tx_elements;
  r_cfg.rx_antenna_mask = cfg_.rx_elements;
  r_cfg.enable_pa_mute = cfg_.pa_mute;

  radio_ = std::make_unique<WifiRadio>(r_cfg);
  if (!radio_->init()) {
    err_msg = "Failed to initialize SDR radio (driver=mipi)";
    tun_->stop();
    return false;
  }

  // 3. Initialize DSP & TX
  DspConfig dsp_cfg;
  dsp_ = std::make_unique<WifiDsp>(dsp_cfg);
  tx_ = std::make_unique<WifiTx>();

  // 4. Initialize Web Server
  server_ = std::make_unique<WifiServer>(cfg_.port, cfg_.web_dir);
  server_->set_msg_callback([this](const ClientMsg& cm) { on_client_command(cm); });
  if (!server_->start(err_msg)) {
    radio_->close();
    tun_->stop();
    return false;
  }

  running_.store(true);
  rx_th_ = std::thread(&WifiLink::rx_loop, this);
  tx_th_ = std::thread(&WifiLink::tx_loop, this);
  srv_th_ = std::thread(&WifiLink::srv_loop, this);
  ping_th_ = std::thread(&WifiLink::ping_loop, this);

  std::cerr << "WifiLink: started on " << hostname_ << " (" << my_ip_
            << ") at " << cfg_.freq_mhz << " MHz, Web: http://0.0.0.0:" << cfg_.port << "\n";
  return true;
}

void WifiLink::stop() {
  if (!running_.load()) return;
  running_.store(false);
  if (tun_) tun_->tx_queue().stop();

  if (rx_th_.joinable()) rx_th_.join();
  if (tx_th_.joinable()) tx_th_.join();
  if (srv_th_.joinable()) srv_th_.join();
  if (ping_th_.joinable()) ping_th_.join();

  if (server_) server_->stop();
  if (radio_) radio_->close();
  if (tun_) tun_->stop();
}

void WifiLink::trigger_ping() {
  bool expected = false;
  if (!ping_.in_progress.compare_exchange_strong(expected, true)) return;
  std::thread(&WifiLink::run_single_ping, this).detach();
}

void WifiLink::run_single_ping() {
  std::string target = peer_ip_;
  {
    std::lock_guard<std::mutex> lock(ping_.mu);
    ping_.status = "Pinging " + target + "...";
  }

  std::string cmd = "ping -c 1 -W 1.0 " + target + " 2>&1";
  FILE* pipe = popen(cmd.c_str(), "r");
  bool ok = false;
  float rtt = 0.f;
  if (pipe) {
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe)) {
      char* t = strstr(buf, "time=");
      if (t) {
        float v = 0.f;
        if (sscanf(t + 5, "%f", &v) == 1) {
          rtt = v;
          ok = true;
        }
      }
    }
    pclose(pipe);
  }

  {
    std::lock_guard<std::mutex> lock(ping_.mu);
    ping_.sent++;
    if (ok) {
      ping_.rcvd++;
      ping_.last_rtt = rtt;
      ping_.sum_rtt += rtt;
      if (ping_.min_rtt == 0.f || rtt < ping_.min_rtt) ping_.min_rtt = rtt;
      if (rtt > ping_.max_rtt) ping_.max_rtt = rtt;
      ping_.avg_rtt = static_cast<float>(ping_.sum_rtt / ping_.rcvd);
      char sbuf[64];
      std::snprintf(sbuf, sizeof(sbuf), "%.1f ms (avg: %.1f ms)", rtt, ping_.avg_rtt);
      ping_.status = sbuf;
    } else {
      ping_.status = "Timeout / Loss";
    }
  }
  ping_.in_progress.store(false, std::memory_order_release);
}

void WifiLink::send_chat(const std::string& text) {
  if (text.empty()) return;
  ChatMessage msg;
  msg.who = hostname_;
  msg.text = text;
  msg.time_ms = now_ms();
  msg.mine = true;

  {
    std::lock_guard<std::mutex> lock(chat_mu_);
    chat_log_.push_back(msg);
    if (chat_log_.size() > 100) chat_log_.erase(chat_log_.begin());
    pending_chat_tx_.push_back(text);
  }

  broadcast_chat();
}

void WifiLink::toggle_tun(bool up) {
  tun_active_.store(up);
  if (tun_ && !tun_->ifname().empty()) {
    std::string cmd = "ip link set dev " + tun_->ifname() + (up ? " up" : " down") + " 2>/dev/null";
    (void)std::system(cmd.c_str());
  }
}

void WifiLink::set_freq(double freq_mhz) {
  cfg_.freq_mhz = freq_mhz;
  if (radio_) radio_->set_freq(freq_mhz);
}

void WifiLink::set_rate(const std::string& rate_name) {
  const RateInfo* r = rate_info_by_name(rate_name);
  if (r) {
    current_rate_.store(r);
    cfg_.rate_str = r->name;
  }
}

void WifiLink::set_tx_gain(double gain_db) {
  cfg_.tx_gain = gain_db;
  if (radio_) radio_->set_tx_gain(gain_db);
}

void WifiLink::set_rx_gain(double gain_db) {
  cfg_.rx_gain = gain_db;
  if (radio_) radio_->set_rx_gain(gain_db);
}

void WifiLink::set_tx_elements(int mask) {
  cfg_.tx_elements = mask & 0xF;
  if (radio_) radio_->set_tx_elements(cfg_.tx_elements);
}

void WifiLink::set_rx_elements(int mask) {
  cfg_.rx_elements = mask & 0xF;
  if (radio_) radio_->set_rx_elements(cfg_.rx_elements);
}

void WifiLink::set_pa_mute(bool enable) {
  cfg_.pa_mute = enable;
  if (radio_) radio_->set_pa_mute(enable);
}

void WifiLink::set_auto_ping(bool enable) {
  auto_ping_.store(enable);
}

void WifiLink::set_viz(bool on) {
  viz_enabled_.store(on);
}

void WifiLink::on_client_command(const ClientMsg& cm) {
  if (cm.type == "set") {
    if (cm.key == "freq") set_freq(cm.val);
    else if (cm.key == "rate") set_rate(cm.str_val);
    else if (cm.key == "tx_gain") set_tx_gain(cm.val);
    else if (cm.key == "rx_gain") set_rx_gain(cm.val);
    else if (cm.key == "tx_ant") set_tx_elements(static_cast<int>(cm.val));
    else if (cm.key == "rx_ant") set_rx_elements(static_cast<int>(cm.val));
    else if (cm.key == "pa_mute") set_pa_mute(cm.bool_val);
    else if (cm.key == "tun") toggle_tun(cm.bool_val);
  } else if (cm.type == "ping") {
    trigger_ping();
  } else if (cm.type == "auto_ping") {
    set_auto_ping(cm.bool_val);
  } else if (cm.type == "chat") {
    send_chat(cm.str_val);
  } else if (cm.type == "viz") {
    set_viz(cm.bool_val);
  } else if (cm.type == "hello") {
    broadcast_status();
    broadcast_chat();
    broadcast_viz();
  } else if (cm.type == "defaults") {
    set_freq(5860.0);
    set_rate("6");
    set_tx_gain(34.0);
    set_rx_gain(30.0);
    set_tx_elements(0x01);
    set_rx_elements(0x01);
    set_pa_mute(true);
    set_auto_ping(false);
    toggle_tun(true);
    broadcast_status();
  }
}

void WifiLink::rx_loop() {
  constexpr size_t kChunkSize = 8192;
  std::vector<c32> rx_buf(kChunkSize);
  std::vector<c32> ring_buf;
  ring_buf.reserve(65536);

  MacRxConfig rx_mac_cfg;
  rx_mac_cfg.my_mac = my_mac_;
  rx_mac_cfg.drop_self_packets = true;

  auto last_psd_time = std::chrono::steady_clock::now();
  uint16_t last_rx_seq = 0xFFFF;
  int64_t last_rx_seq_time_ms = 0;
  size_t pending_needed_ring_sz = 0;

  while (running_.load()) {
    int n = radio_->read_rx(rx_buf.data(), kChunkSize, 30000);
    if (n <= 0) continue;

    // Periodically compute ADC Spectrum PSD from raw samples
    auto now_clock = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_clock - last_psd_time).count() >= 0.1) {
      last_psd_time = now_clock;
      std::vector<float> psd(64, 0.f);
      c32 fft_out[64];
      for (size_t off = 0; off + 64 <= static_cast<size_t>(n); off += 64) {
        dsp_->fft64(&rx_buf[off], fft_out);
        for (int k = 0; k < 64; ++k) {
          psd[static_cast<size_t>(k)] += std::norm(fft_out[k]);
        }
      }
      std::vector<float> psd_db(64);
      for (int k = 0; k < 64; ++k) {
        // fftshifted
        int shifted = (k + 32) % 64;
        psd_db[static_cast<size_t>(k)] = 10.f * std::log10(psd[static_cast<size_t>(shifted)] + 1e-9f) - 30.f;
      }
      std::lock_guard<std::mutex> lock(viz_mu_);
      adc_spec_psd_ = std::move(psd_db);
      const size_t trace_len = std::min<size_t>(200, static_cast<size_t>(n));
      adc_iq_trace_.assign(rx_buf.begin(), rx_buf.begin() + trace_len);
      if (FILE* fp = std::fopen("/dev/shm/last_rx_chunk.cf32.tmp", "wb")) {
        std::fwrite(rx_buf.data(), sizeof(c32), static_cast<size_t>(n), fp);
        std::fclose(fp);
        std::rename("/dev/shm/last_rx_chunk.cf32.tmp", "/dev/shm/last_rx_chunk.cf32");
      }
    }

    ring_buf.insert(ring_buf.end(), rx_buf.begin(), rx_buf.begin() + n);
    if (ring_buf.size() > 65536) {
      ring_buf.erase(ring_buf.begin(), ring_buf.begin() + 8192);
      pending_needed_ring_sz = 0;
    }

    if (pending_needed_ring_sz > 0 && ring_buf.size() < pending_needed_ring_sz && ring_buf.size() < 60000) {
      last_rx_frame_ms_.store(now_ms(), std::memory_order_release);
      continue;
    }

    while (ring_buf.size() >= 400) {
      pending_needed_ring_sz = 0;
      StsResult sts = dsp_->detect_sts(ring_buf.data(), ring_buf.size());
      if (sts.trig_idx < 0) {
        if (ring_buf.size() > 48) {
          ring_buf.erase(ring_buf.begin(), ring_buf.end() - 48);
        }
        break;
      }

      const size_t burst_start = (sts.trig_idx >= 32) ? static_cast<size_t>(sts.trig_idx - 32) : 0;
      if (burst_start + 800 > ring_buf.size()) {
        pending_needed_ring_sz = burst_start + 800;
        last_rx_frame_ms_.store(now_ms(), std::memory_order_release);
        break;
      }

      std::span<const c32> burst_span(&ring_buf[burst_start], ring_buf.size() - burst_start);
      FrameResult fr = dsp_->decode_burst(burst_span, &sts);

      if (fr.signal_ok && fr.needed_samples > 0 &&
          burst_start + static_cast<size_t>(fr.needed_samples) > ring_buf.size()) {
        pending_needed_ring_sz = burst_start + static_cast<size_t>(fr.needed_samples);
        last_rx_frame_ms_.store(now_ms(), std::memory_order_release);
        break;
      }

      std::fprintf(stderr, "[STS-TRIG] idx=%d m=%.2f pwr=%.4f ring_sz=%zu\n",
                   sts.trig_idx, sts.metric_at_trig, sts.R_at_trig / 16.f, ring_buf.size());

      if (fr.signal_ok) {
        if (fr.payload.size() >= 16 &&
            std::memcmp(&fr.payload[10], my_mac_.data(), 6) == 0) {
          const size_t burst_len = (fr.needed_samples > 0)
                                       ? static_cast<size_t>(fr.needed_samples)
                                       : (400 + static_cast<size_t>(fr.pkt_len * 8));
          const size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
          ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
          continue;
        }
        if (FILE* fp = std::fopen("/dev/shm/last_burst.cf32.tmp", "wb")) {
          const size_t dump_n = std::min<size_t>(burst_span.size(), 4800);
          std::fwrite(burst_span.data(), sizeof(c32), dump_n, fp);
          std::fclose(fp);
          std::rename("/dev/shm/last_burst.cf32.tmp", "/dev/shm/last_burst.cf32");
        }
        std::fprintf(stderr, "[RX-BURST] trig=%d fcs_ok=%d len=%zu snr=%.1f evm=%.1f cfo=%.0f peak=%d\n",
                     sts.trig_idx, (int)fr.fcs_ok, fr.payload.size(), fr.snr_db, fr.evm_db, fr.fine_cfo_hz, fr.peak_cs8);
        rx_frames_.fetch_add(1, std::memory_order_relaxed);
        if (fr.fcs_ok) {
          fcs_ok_.fetch_add(1, std::memory_order_relaxed);
          rx_bytes_.fetch_add(static_cast<uint64_t>(fr.pkt_len), std::memory_order_relaxed);
          last_rx_frame_ms_.store(now_ms(), std::memory_order_release);

          last_snr_.store(fr.snr_db, std::memory_order_relaxed);
          last_evm_.store(fr.evm_db, std::memory_order_relaxed);
          last_cfo_.store(fr.fine_cfo_hz, std::memory_order_relaxed);
          last_peak_adc_.store(fr.peak_cs8, std::memory_order_relaxed);

          uint16_t piggy_ack = 0xFFFFu;
          if (WifiMac::extract_piggy_ack(fr.payload, piggy_ack)) {
            last_rx_ack_seq_.store(piggy_ack, std::memory_order_release);
          }

          uint16_t standalone_ack = 0xFFFFu;
          if (WifiMac::decapsulate_ack(fr.payload, standalone_ack, rx_mac_cfg)) {
            last_rx_ack_seq_.store(standalone_ack, std::memory_order_release);
            std::fprintf(stderr, "[RX-ACK] ack_seq=%u snr=%.1f evm=%.1f\n",
                         (unsigned)standalone_ack, fr.snr_db, fr.evm_db);
            {
              std::lock_guard<std::mutex> lock(viz_mu_);
              last_frame_viz_ = fr;
            }
            const size_t burst_len = (fr.needed_samples > 0)
                                         ? static_cast<size_t>(fr.needed_samples)
                                         : (400 + static_cast<size_t>(fr.pkt_len * 8));
            const size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
            ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
            continue;
          }

          uint16_t rx_seq = 0xFFFF;
          if (fr.payload.size() >= 24) {
            rx_seq = (static_cast<uint16_t>(fr.payload[22]) | (static_cast<uint16_t>(fr.payload[23]) << 8)) >> 4;
          }
          const int64_t cur_now = now_ms();
          const int64_t dup_age_ms = cur_now - last_rx_seq_time_ms;
          const bool is_dup = (rx_seq != 0xFFFF && rx_seq == last_rx_seq && dup_age_ms < 1500);
          if (!is_dup || dup_age_ms >= 45) {
            last_rx_seq = rx_seq;
            last_rx_seq_time_ms = cur_now;
          }

          bool need_ack = false;
          if (is_dup) {
            std::fprintf(stderr, "[RX-DUP] seq=%u dropped duplicate within %lld ms\n",
                         (unsigned)rx_seq, (long long)dup_age_ms);
            // If dup_age_ms >= 45 ms, this is an ARQ retransmission burst from the peer
            // (not the 19.34 ms second VSYNC scanout of the original burst), so re-ACK it.
            if (dup_age_ms >= 45) {
              need_ack = true;
              peer_retransmitted_.store(true, std::memory_order_release);
            }
          } else {
            // 1. Check for Chat packet
            std::string chat_sender, chat_text;
            if (WifiMac::decapsulate_chat(fr.payload, chat_sender, chat_text, rx_mac_cfg)) {
              need_ack = true;
              std::cerr << "RX CHAT from " << chat_sender << ": " << chat_text << " (snr=" << fr.snr_db << " dB)\n";
              ChatMessage cm;
              cm.who = chat_sender;
              cm.text = chat_text;
              cm.time_ms = now_ms();
              cm.mine = false;
              {
                std::lock_guard<std::mutex> lock(chat_mu_);
                chat_log_.push_back(cm);
                if (chat_log_.size() > 100) chat_log_.erase(chat_log_.begin());
              }
              peer_hostname_ = chat_sender;
              broadcast_chat();
            } else {
              // 2. Check for IPv4 TUN packet(s) (supports single or coalesced IPv4 packets)
              auto ip_pkt = WifiMac::decapsulate_ipv4(fr.payload, rx_mac_cfg);
              if (!ip_pkt.empty()) {
                need_ack = true;
                size_t off = 0;
                while (off + 20 <= ip_pkt.size() && (ip_pkt[off] >> 4) == 4) {
                  const uint16_t sub_len =
                      (static_cast<uint16_t>(ip_pkt[off + 2]) << 8) | ip_pkt[off + 3];
                  if (sub_len < 20 || off + sub_len > ip_pkt.size()) break;
                  const uint8_t* p = &ip_pkt[off];
                  char s_ip[20] = "-", d_ip[20] = "-";
                  std::snprintf(s_ip, sizeof(s_ip), "%u.%u.%u.%u", p[12], p[13], p[14], p[15]);
                  std::snprintf(d_ip, sizeof(d_ip), "%u.%u.%u.%u", p[16], p[17], p[18], p[19]);
                  if (std::string(s_ip) != my_ip_) {
                    peer_ip_ = s_ip;
                  }
                  std::cerr << "RX IP PKT len=" << sub_len << " src=" << s_ip << " dst=" << d_ip
                            << " (snr=" << fr.snr_db << " dB)\n";
                  if (tun_active_.load(std::memory_order_relaxed)) {
                    tun_->inject_packet(p, sub_len);
                  }
                  off += sub_len;
                }
              } else {
                static auto last_other_log = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration<double>(now - last_other_log).count() >= 2.0) {
                  last_other_log = now;
                  std::fprintf(stderr, "RX other 802.11 frame: len=%zu sa=%02x:%02x:%02x:%02x:%02x:%02x da=%02x:%02x:%02x:%02x:%02x:%02x snr=%.1f\n",
                               fr.payload.size(),
                               fr.payload.size() >= 16 ? fr.payload[10] : 0,
                               fr.payload.size() >= 16 ? fr.payload[11] : 0,
                               fr.payload.size() >= 16 ? fr.payload[12] : 0,
                               fr.payload.size() >= 16 ? fr.payload[13] : 0,
                               fr.payload.size() >= 16 ? fr.payload[14] : 0,
                               fr.payload.size() >= 16 ? fr.payload[15] : 0,
                               fr.payload.size() >= 10 ? fr.payload[4] : 0,
                               fr.payload.size() >= 10 ? fr.payload[5] : 0,
                               fr.payload.size() >= 10 ? fr.payload[6] : 0,
                               fr.payload.size() >= 10 ? fr.payload[7] : 0,
                               fr.payload.size() >= 10 ? fr.payload[8] : 0,
                               fr.payload.size() >= 10 ? fr.payload[9] : 0,
                               fr.snr_db);
                }
              }
            }
          }

          if (need_ack && rx_seq != 0xFFFFu) {
            last_peer_data_seq_.store(rx_seq, std::memory_order_release);
            ack_to_send_seq_.store(rx_seq, std::memory_order_release);
            ack_pending_.store(true, std::memory_order_release);
          }

          {
            std::lock_guard<std::mutex> lock(viz_mu_);
            last_frame_viz_ = fr;
          }

          const size_t burst_len = (fr.needed_samples > 0)
                                       ? static_cast<size_t>(fr.needed_samples)
                                       : (400 + static_cast<size_t>(fr.pkt_len * 8));
          size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
          ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
        } else {
          last_rx_frame_ms_.store(now_ms(), std::memory_order_release);
          fcs_fail_.fetch_add(1, std::memory_order_relaxed);
          std::fprintf(stderr, "[FCS-FAIL] trig=%d len=%d snr=%.1f evm=%.1f\n",
                       sts.trig_idx, fr.pkt_len, fr.snr_db, fr.evm_db);
          const size_t burst_len = (fr.needed_samples > 0)
                                       ? static_cast<size_t>(fr.needed_samples)
                                       : 800;
          size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
          ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
        }
      } else {
        std::fprintf(stderr, "[SIG-FAIL] trig=%d ltf1=%d ltf_peak=%.2f ring_sz=%zu\n",
                     sts.trig_idx, fr.ltf1_idx, fr.ltf_peak, ring_buf.size());
        const size_t advance = std::min<size_t>(static_cast<size_t>(sts.trig_idx) + 160, ring_buf.size());
        ring_buf.erase(ring_buf.begin(), ring_buf.begin() + advance);
      }
    }
  }
}

void WifiLink::tx_loop() {
  uint16_t seq_num = static_cast<uint16_t>(now_ms() & 0x0FFFu);
  MacTxConfig tx_cfg;
  tx_cfg.sa = my_mac_;
  tx_cfg.da = peer_mac_;

  auto wait_turnaround = [this]() {
    // 1. TDD Turnaround Guard: Wait in a loop until at least 40 ms have elapsed since the
    // last received burst (including any second VSYNC scanout) and the RF channel is clear
    // so the peer has completed its PA unmute window + zero-tail VSYNC flip +
    // CSI-2 DMA drain before we start transmitting.
    while (running_.load()) {
      const int64_t last_rx = last_rx_frame_ms_.load(std::memory_order_acquire);
      const int64_t rx_age = now_ms() - last_rx;
      if (rx_age >= 0 && rx_age < 40) {
        std::this_thread::sleep_for(std::chrono::milliseconds(40 - rx_age));
        continue;
      }
      if (!radio_->is_channel_clear()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        continue;
      }
      break;
    }

    // 2. Inter-Frame Spacing Guard: Ensure our own previous TX burst has finished settling.
    while (running_.load()) {
      const int64_t last_tx = last_tx_frame_ms_.load(std::memory_order_acquire);
      const int64_t tx_age = now_ms() - last_tx;
      if (tx_age >= 0 && tx_age < 10) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10 - tx_age));
      } else {
        break;
      }
    }
  };

  auto coalesce_tun_packets = [this](std::vector<uint8_t>& buf) {
    while (buf.size() < 600) {
      std::vector<uint8_t> next_pkt;
      if (!tun_->tx_queue().try_pop_if_fits(next_pkt, 650 - buf.size())) break;
      if (next_pkt.size() < 20 || (next_pkt[0] >> 4) != 4) continue;
      buf.insert(buf.end(), next_pkt.begin(), next_pkt.end());
    }
  };

  auto send_chat_with_arq = [&](const std::string& text) {
    const uint16_t my_seq = (seq_num++) & 0x0FFFu;
    constexpr int kMaxAttempts = 7;
    const int64_t base_timeout_ms = (my_mac_[5] == 2) ? 220 : 320;
    const int64_t step_timeout_ms = (my_mac_[5] == 2) ? 40 : 95;

    for (int attempt = 0; attempt < kMaxAttempts && running_.load(); ++attempt) {
      wait_turnaround();
      MacTxConfig cfg = tx_cfg;
      cfg.retry = (attempt > 0);
      if (ack_pending_.exchange(false, std::memory_order_acq_rel)) {
        cfg.has_piggy_ack = true;
        cfg.piggy_ack_seq = ack_to_send_seq_.load(std::memory_order_acquire);
      } else if (last_peer_data_seq_.load(std::memory_order_acquire) != 0xFFFFu) {
        cfg.has_piggy_ack = true;
        cfg.piggy_ack_seq = last_peer_data_seq_.load(std::memory_order_acquire);
      }
      peer_retransmitted_.store(false, std::memory_order_release);

      auto mpdu = WifiMac::encapsulate_chat(hostname_, text, my_seq, cfg);
      const RateInfo* r = current_rate_.load(std::memory_order_relaxed);
      auto cs8 = tx_->modulate_frame_cs8(mpdu, *r);
      if (cs8.empty()) break;

      radio_->write_tx_cs8(cs8.data(), cs8.size() / 2, 50000);
      last_tx_frame_ms_.store(now_ms(), std::memory_order_release);
      tx_packets_.fetch_add(1, std::memory_order_relaxed);
      tx_bytes_.fetch_add(mpdu.size(), std::memory_order_relaxed);
      if (attempt > 0) {
        std::fprintf(stderr, "[ARQ-RETRY] chat seq=%u attempt=%d/%d\n",
                     (unsigned)my_seq, attempt + 1, kMaxAttempts);
      }

      const int64_t jitter = (now_ms() * 7 + attempt * 13) % 23;
      int64_t deadline = now_ms() + base_timeout_ms + attempt * step_timeout_ms + jitter;
      bool acked = false;
      while (running_.load() && now_ms() < deadline) {
        if (last_rx_ack_seq_.load(std::memory_order_acquire) == my_seq) {
          acked = true;
          break;
        }
        const int64_t rx_age = now_ms() - last_rx_frame_ms_.load(std::memory_order_acquire);
        if ((rx_age >= 0 && rx_age < 40) || !radio_->is_channel_clear()) {
          deadline = std::max(deadline, now_ms() + 45);
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          continue;
        }
        if (peer_retransmitted_.exchange(false, std::memory_order_acq_rel) ||
            ack_pending_.load(std::memory_order_acquire)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (acked) break;
    }
  };

  auto send_ip_with_arq = [&](std::vector<uint8_t>& ip_payload) {
    const uint16_t my_seq = (seq_num++) & 0x0FFFu;
    constexpr int kMaxAttempts = 7;
    const int64_t base_timeout_ms = (my_mac_[5] == 2) ? 220 : 320;
    const int64_t step_timeout_ms = (my_mac_[5] == 2) ? 40 : 95;

    for (int attempt = 0; attempt < kMaxAttempts && running_.load(); ++attempt) {
      wait_turnaround();
      if (attempt == 0) {
        coalesce_tun_packets(ip_payload);
      }

      MacTxConfig cfg = tx_cfg;
      cfg.retry = (attempt > 0);
      if (ack_pending_.exchange(false, std::memory_order_acq_rel)) {
        cfg.has_piggy_ack = true;
        cfg.piggy_ack_seq = ack_to_send_seq_.load(std::memory_order_acquire);
      } else if (last_peer_data_seq_.load(std::memory_order_acquire) != 0xFFFFu) {
        cfg.has_piggy_ack = true;
        cfg.piggy_ack_seq = last_peer_data_seq_.load(std::memory_order_acquire);
      }
      peer_retransmitted_.store(false, std::memory_order_release);

      auto mpdu = WifiMac::encapsulate_ipv4(ip_payload, my_seq, cfg);
      const RateInfo* r = current_rate_.load(std::memory_order_relaxed);
      auto cs8 = tx_->modulate_frame_cs8(mpdu, *r);
      if (cs8.empty()) break;

      radio_->write_tx_cs8(cs8.data(), cs8.size() / 2, 50000);
      last_tx_frame_ms_.store(now_ms(), std::memory_order_release);
      tx_packets_.fetch_add(1, std::memory_order_relaxed);
      tx_bytes_.fetch_add(ip_payload.size(), std::memory_order_relaxed);
      if (attempt > 0) {
        std::fprintf(stderr, "[ARQ-RETRY] ip seq=%u attempt=%d/%d len=%zu\n",
                     (unsigned)my_seq, attempt + 1, kMaxAttempts, ip_payload.size());
      }

      const int64_t jitter = (now_ms() * 7 + attempt * 13) % 23;
      int64_t deadline = now_ms() + base_timeout_ms + attempt * step_timeout_ms + jitter;
      bool acked = false;
      while (running_.load() && now_ms() < deadline) {
        if (last_rx_ack_seq_.load(std::memory_order_acquire) == my_seq) {
          acked = true;
          break;
        }
        const int64_t rx_age = now_ms() - last_rx_frame_ms_.load(std::memory_order_acquire);
        if ((rx_age >= 0 && rx_age < 40) || !radio_->is_channel_clear()) {
          deadline = std::max(deadline, now_ms() + 45);
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          continue;
        }
        if (peer_retransmitted_.exchange(false, std::memory_order_acq_rel) ||
            ack_pending_.load(std::memory_order_acquire)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (acked) break;
    }
  };

  while (running_.load()) {
    // 1. Process pending chat message
    std::string chat_to_send;
    {
      std::lock_guard<std::mutex> lock(chat_mu_);
      if (!pending_chat_tx_.empty()) {
        chat_to_send = pending_chat_tx_.front();
        pending_chat_tx_.erase(pending_chat_tx_.begin());
      }
    }

    if (!chat_to_send.empty()) {
      send_chat_with_arq(chat_to_send);
      continue;
    }

    // 2. Process TUN IP packets
    std::vector<uint8_t> ip_packet;
    if (tun_->tx_queue().pop(ip_packet, 4)) {
      if (!ip_packet.empty() && tun_active_.load(std::memory_order_relaxed) &&
          ip_packet.size() >= 20 && (ip_packet[0] >> 4) == 4) {
        send_ip_with_arq(ip_packet);
        continue;
      }
    }

    // 3. If a Link-Layer ACK is pending for a received Data frame, wait for TDD turnaround;
    // if an IP packet arrived during turnaround, piggyback the ACK on that Data frame,
    // otherwise transmit a compact Standalone ACK frame.
    if (ack_pending_.load(std::memory_order_acquire)) {
      wait_turnaround();
      std::vector<uint8_t> piggy_ip;
      if (tun_active_.load(std::memory_order_relaxed) && tun_->tx_queue().pop(piggy_ip, 0) &&
          piggy_ip.size() >= 20 && (piggy_ip[0] >> 4) == 4) {
        send_ip_with_arq(piggy_ip);
        continue;
      }
      if (ack_pending_.exchange(false, std::memory_order_acq_rel)) {
        const uint16_t ack_seq = ack_to_send_seq_.load(std::memory_order_acquire);
        const uint16_t ack_frame_seq = (seq_num++) & 0x0FFFu;
        auto mpdu = WifiMac::encapsulate_ack(ack_seq, ack_frame_seq, tx_cfg);
        const RateInfo* r = current_rate_.load(std::memory_order_relaxed);
        auto cs8 = tx_->modulate_frame_cs8(mpdu, *r);
        if (!cs8.empty()) {
          radio_->write_tx_cs8(cs8.data(), cs8.size() / 2, 50000);
          last_tx_frame_ms_.store(now_ms(), std::memory_order_release);
        }
      }
    }
  }
}

void WifiLink::ping_loop() {
  while (running_.load()) {
    for (int i = 0; i < 25 && running_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!running_.load()) break;
    if (auto_ping_.load()) {
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - ping_.last_ping_time).count() >= 1.5) {
        ping_.last_ping_time = now;
        trigger_ping();
      }
    }
  }
}

void WifiLink::srv_loop() {
  auto last_st_time = std::chrono::steady_clock::now();
  auto last_viz_time = std::chrono::steady_clock::now();

  uint64_t last_rx_b = 0, last_tx_b = 0;
  auto last_rate_clock = std::chrono::steady_clock::now();
  double cur_gp_rx = 0.0, cur_gp_tx = 0.0;

  while (running_.load()) {
    server_->poll(10);

    auto now = std::chrono::steady_clock::now();
    double dt_rate = std::chrono::duration<double>(now - last_rate_clock).count();
    if (dt_rate >= 0.5) {
      uint64_t cur_rx_b = rx_bytes_.load(std::memory_order_relaxed);
      uint64_t cur_tx_b = tx_bytes_.load(std::memory_order_relaxed);
      cur_gp_rx = (cur_rx_b - last_rx_b) * 8.0 / (dt_rate * 1000.0);
      cur_gp_tx = (cur_tx_b - last_tx_b) * 8.0 / (dt_rate * 1000.0);
      last_goodput_rx_.store(static_cast<float>(cur_gp_rx), std::memory_order_relaxed);
      last_goodput_tx_.store(static_cast<float>(cur_gp_tx), std::memory_order_relaxed);
      last_rx_b = cur_rx_b;
      last_tx_b = cur_tx_b;
      last_rate_clock = now;
    }

    if (std::chrono::duration<double>(now - last_st_time).count() >= 0.1) {
      last_st_time = now;
      broadcast_status();
    }

    if (viz_enabled_.load() && std::chrono::duration<double>(now - last_viz_time).count() >= 0.07) {
      last_viz_time = now;
      broadcast_viz();
    }
  }
}

void WifiLink::broadcast_status() {
  LinkStatus st;
  st.host = hostname_;
  st.peer_host = peer_hostname_.empty() ? "peer" : peer_hostname_;
  st.my_ip = my_ip_;
  st.peer_ip = peer_ip_;
  st.my_mac = WifiMac::format_mac(my_mac_);
  st.peer_mac = WifiMac::format_mac(peer_mac_);

  int64_t last_rx = last_rx_frame_ms_.load(std::memory_order_acquire);
  st.link_up = (last_rx > 0 && (now_ms() - last_rx < 3000));
  st.tun_up = tun_active_.load(std::memory_order_relaxed);
  st.auto_ping = auto_ping_.load();
  st.viz_on = viz_enabled_.load();

  st.freq_mhz = cfg_.freq_mhz;
  const RateInfo* r = current_rate_.load(std::memory_order_relaxed);
  st.rate_name = r->name;
  st.rate_val = r->rate_val;
  st.tx_gain = cfg_.tx_gain;
  st.rx_gain = cfg_.rx_gain;
  st.tx_elements = cfg_.tx_elements;
  st.rx_elements = cfg_.rx_elements;
  st.pa_mute = cfg_.pa_mute;

  st.tx_packets = tx_packets_.load(std::memory_order_relaxed);
  st.rx_frames = rx_frames_.load(std::memory_order_relaxed);
  st.fcs_ok = fcs_ok_.load(std::memory_order_relaxed);
  st.fcs_fail = fcs_fail_.load(std::memory_order_relaxed);
  const uint64_t total_fcs = st.fcs_ok + st.fcs_fail;
  st.delivery_pct = (total_fcs > 0) ? (100.f * st.fcs_ok / total_fcs) : 0.f;

  st.snr_db = last_snr_.load(std::memory_order_relaxed);
  st.evm_db = last_evm_.load(std::memory_order_relaxed);
  st.cfo_hz = last_cfo_.load(std::memory_order_relaxed);
  st.peak_adc_counts = last_peak_adc_.load(std::memory_order_relaxed);
  st.goodput_rx_kbps = last_goodput_rx_.load(std::memory_order_relaxed);
  st.goodput_tx_kbps = last_goodput_tx_.load(std::memory_order_relaxed);

  {
    std::lock_guard<std::mutex> lock(ping_.mu);
    st.ping_rtt_ms = ping_.last_rtt;
    st.ping_min_ms = ping_.min_rtt;
    st.ping_max_ms = ping_.max_rtt;
    st.ping_avg_ms = ping_.avg_rtt;
    st.ping_sent = ping_.sent;
    st.ping_rcvd = ping_.rcvd;
    st.ping_status = ping_.status;
  }

  std::ostringstream ss;
  ss << "{\"t\":\"st\",\"host\":\"" << st.host << "\",\"peer_host\":\"" << st.peer_host
     << "\",\"my_ip\":\"" << st.my_ip << "\",\"peer_ip\":\"" << st.peer_ip
     << "\",\"my_mac\":\"" << st.my_mac << "\",\"peer_mac\":\"" << st.peer_mac
     << "\",\"link_up\":" << (st.link_up ? "true" : "false")
     << ",\"tun_up\":" << (st.tun_up ? "true" : "false")
     << ",\"auto_ping\":" << (st.auto_ping ? "true" : "false")
     << ",\"viz_on\":" << (st.viz_on ? "true" : "false")
     << ",\"freq\":" << std::fixed << std::setprecision(1) << st.freq_mhz
     << ",\"rate\":\"" << st.rate_name << "\",\"rate_val\":" << st.rate_val
     << ",\"tx_gain\":" << std::setprecision(1) << st.tx_gain
     << ",\"rx_gain\":" << std::setprecision(1) << st.rx_gain
     << ",\"tx_elements\":" << st.tx_elements
     << ",\"rx_elements\":" << st.rx_elements
     << ",\"pa_mute\":" << (st.pa_mute ? "true" : "false")
     << ",\"tx_packets\":" << st.tx_packets
     << ",\"rx_frames\":" << st.rx_frames
     << ",\"fcs_ok\":" << st.fcs_ok
     << ",\"fcs_fail\":" << st.fcs_fail
     << ",\"tx_bytes\":" << tx_bytes_.load(std::memory_order_relaxed)
     << ",\"rx_bytes\":" << rx_bytes_.load(std::memory_order_relaxed)
     << ",\"gp_rx\":" << std::setprecision(1) << st.goodput_rx_kbps
     << ",\"gp_tx\":" << std::setprecision(1) << st.goodput_tx_kbps
     << ",\"goodput_kbps\":" << std::setprecision(1) << (st.goodput_rx_kbps + st.goodput_tx_kbps)
     << ",\"delivery\":" << std::setprecision(1) << st.delivery_pct
     << ",\"snr\":" << std::setprecision(1) << st.snr_db
     << ",\"evm\":" << std::setprecision(1) << st.evm_db
     << ",\"cfo\":" << std::setprecision(1) << st.cfo_hz
     << ",\"peak_adc\":" << st.peak_adc_counts
     << ",\"ping_rtt\":" << std::setprecision(1) << st.ping_rtt_ms
     << ",\"ping_min\":" << std::setprecision(1) << st.ping_min_ms
     << ",\"ping_max\":" << std::setprecision(1) << st.ping_max_ms
     << ",\"ping_avg\":" << std::setprecision(1) << st.ping_avg_ms
     << ",\"ping_sent\":" << st.ping_sent
     << ",\"ping_rcvd\":" << st.ping_rcvd
     << ",\"ping_status\":\"" << st.ping_status << "\"}";

  server_->broadcast_json(ss.str());
}

void WifiLink::broadcast_viz() {
  std::ostringstream ss;
  ss << "{\"t\":\"viz\"";

  {
    std::lock_guard<std::mutex> lock(viz_mu_);

    // 1. Stage 1: ADC Spectrum & I/Q
    if (!adc_spec_psd_.empty()) {
      ss << ",\"adc_psd\":[";
      for (size_t i = 0; i < adc_spec_psd_.size(); ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(1) << adc_spec_psd_[i];
      }
      ss << "]";
    }
    if (!adc_iq_trace_.empty()) {
      ss << ",\"adc_iq\":[";
      for (size_t i = 0; i < adc_iq_trace_.size(); ++i) {
        if (i > 0) ss << ",";
        ss << static_cast<int>(std::round(adc_iq_trace_[i].real() * 127.f)) << ","
           << static_cast<int>(std::round(adc_iq_trace_[i].imag() * 127.f));
      }
      ss << "]";
    }

    // 2. Stage 2: Time domain preamble / envelope
    if (!last_frame_viz_.time_iq.empty()) {
      ss << ",\"sts_iq\":[";
      for (size_t i = 0; i < last_frame_viz_.time_iq.size(); ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(3) << last_frame_viz_.time_iq[i].real() << ","
           << std::setprecision(3) << last_frame_viz_.time_iq[i].imag();
      }
      ss << "],\"ltf_peak\":" << std::setprecision(3) << last_frame_viz_.ltf_peak;
    }

    // 3. Stage 3: Channel Response H(f)
    if (!last_frame_viz_.H_mag.empty()) {
      ss << ",\"h_mag\":[";
      for (size_t i = 0; i < last_frame_viz_.H_mag.size(); ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(1) << last_frame_viz_.H_mag[i];
      }
      ss << "],\"h_phase\":[";
      for (size_t i = 0; i < last_frame_viz_.H_phase.size(); ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(1) << last_frame_viz_.H_phase[i];
      }
      ss << "]";
    }

    // 4. Stage 4: Phase tracking
    if (!last_frame_viz_.pilot_cpe.empty()) {
      ss << ",\"phase_cpe\":[";
      for (size_t i = 0; i < last_frame_viz_.pilot_cpe.size(); ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(3) << last_frame_viz_.pilot_cpe[i];
      }
      ss << "]";
    }

    // 5. Stage 5: Equalized constellation
    if (!last_frame_viz_.eq_symbols.empty()) {
      ss << ",\"constel\":[";
      const size_t max_pts = std::min<size_t>(384, last_frame_viz_.eq_symbols.size());
      for (size_t i = 0; i < max_pts; ++i) {
        if (i > 0) ss << ",";
        ss << std::fixed << std::setprecision(3) << last_frame_viz_.eq_symbols[i].real() << ","
           << std::setprecision(3) << last_frame_viz_.eq_symbols[i].imag();
      }
      ss << "]";
      ss << ",\"pkt_len\":" << last_frame_viz_.pkt_len
         << ",\"coarse_cfo\":" << std::setprecision(1) << last_frame_viz_.coarse_cfo_hz
         << ",\"fine_cfo\":" << std::setprecision(1) << last_frame_viz_.fine_cfo_hz;
    }
  }

  ss << "}";
  server_->broadcast_json(ss.str());
}

void WifiLink::broadcast_chat() {
  std::ostringstream ss;
  ss << "{\"t\":\"chat\",\"msgs\":[";
  {
    std::lock_guard<std::mutex> lock(chat_mu_);
    for (size_t i = 0; i < chat_log_.size(); ++i) {
      if (i > 0) ss << ",";
      const auto& m = chat_log_[i];
      ss << "{\"who\":\"" << m.who << "\",\"text\":\"" << m.text
         << "\",\"time\":" << m.time_ms << ",\"mine\":" << (m.mine ? "true" : "false") << "}";
    }
  }
  ss << "]}";
  server_->broadcast_json(ss.str());
}

}  // namespace quadrf_wifi
