#pragma once

#include "wifi_constants.hpp"
#include "wifi_dsp.hpp"
#include "wifi_mac.hpp"
#include "wifi_radio.hpp"
#include "wifi_server.hpp"
#include "wifi_tun.hpp"
#include "wifi_tx.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace quadrf_wifi {

struct ChatMessage {
  std::string who;
  std::string text;
  int64_t time_ms = 0;
  bool mine = false;
};

struct LinkStatus {
  std::string host = "quadrf";
  std::string peer_host = "peer";
  std::string my_ip = "10.99.0.1";
  std::string peer_ip = "10.99.0.2";
  std::string my_mac = "02:00:0a:63:00:01";
  std::string peer_mac = "02:00:0a:63:00:02";

  bool link_up = false;
  bool tun_up = true;
  bool auto_ping = false;
  bool viz_on = true;

  double freq_mhz = 5860.0;
  std::string rate_name = "6";
  int rate_val = 0;
  double tx_gain = 34.0;
  double rx_gain = 30.0;
  int tx_elements = 0x01;
  int rx_elements = 0x01;
  bool pa_mute = true;

  double goodput_rx_kbps = 0.0;
  double goodput_tx_kbps = 0.0;
  uint64_t tx_packets = 0;
  uint64_t rx_frames = 0;
  uint64_t fcs_ok = 0;
  uint64_t fcs_fail = 0;
  float delivery_pct = 0.0f;

  float snr_db = 0.0f;
  float evm_db = 0.0f;
  float cfo_hz = 0.0f;
  int peak_adc_counts = 0;

  float ping_rtt_ms = 0.0f;
  float ping_min_ms = 0.0f;
  float ping_max_ms = 0.0f;
  float ping_avg_ms = 0.0f;
  uint32_t ping_sent = 0;
  uint32_t ping_rcvd = 0;
  std::string ping_status = "Idle";
};

class WifiLink {
 public:
  struct Config {
    int port = 8092;
    std::string web_dir = "";
    double freq_mhz = 5860.0;
    std::string rate_str = "6";
    double tx_gain = 34.0;
    double rx_gain = 30.0;
    int tx_elements = 0x01;
    int rx_elements = 0x01;
    bool pa_mute = true;
    std::string tun_name = "tun0";
    std::string ip_cidr = "";
    int mtu = 576;
  };

  WifiLink();
  explicit WifiLink(Config cfg);
  ~WifiLink();

  WifiLink(const WifiLink&) = delete;
  WifiLink& operator=(const WifiLink&) = delete;

  bool start(std::string& err_msg);
  void stop();
  bool is_running() const { return running_.load(); }

  void trigger_ping();
  void send_chat(const std::string& text);
  void toggle_tun(bool up);

  void set_freq(double freq_mhz);
  void set_rate(const std::string& rate_name);
  void set_tx_gain(double gain_db);
  void set_rx_gain(double gain_db);
  void set_tx_elements(int mask);
  void set_rx_elements(int mask);
  void set_pa_mute(bool enable);
  void set_auto_ping(bool enable);
  void set_viz(bool on);

 private:
  Config cfg_;
  std::atomic<bool> running_{false};

  std::unique_ptr<WifiRadio> radio_;
  std::unique_ptr<WifiDsp> dsp_;
  std::unique_ptr<WifiTx> tx_;
  std::unique_ptr<WifiTun> tun_;
  std::unique_ptr<WifiServer> server_;

  MacAddr my_mac_{0};
  MacAddr peer_mac_{0};
  std::string my_ip_;
  std::string peer_ip_;
  std::string hostname_;
  std::string peer_hostname_;

  std::atomic<const RateInfo*> current_rate_{&kRate6};
  std::atomic<bool> tun_active_{true};
  std::atomic<bool> auto_ping_{false};
  std::atomic<bool> viz_enabled_{true};
  std::atomic<int64_t> last_rx_frame_ms_{0};
  std::atomic<int64_t> last_tx_frame_ms_{0};
  std::atomic<uint16_t> last_rx_ack_seq_{0xFFFFu};
  std::atomic<uint16_t> last_peer_data_seq_{0xFFFFu};
  std::atomic<uint16_t> ack_to_send_seq_{0xFFFFu};
  std::atomic<bool> ack_pending_{false};
  std::atomic<bool> peer_retransmitted_{false};

  // Telemetry counters
  std::atomic<uint64_t> tx_packets_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> fcs_ok_{0};
  std::atomic<uint64_t> fcs_fail_{0};
  std::atomic<uint64_t> tx_bytes_{0};
  std::atomic<uint64_t> rx_bytes_{0};

  std::atomic<float> last_snr_{0.f};
  std::atomic<float> last_evm_{0.f};
  std::atomic<float> last_cfo_{0.f};
  std::atomic<int> last_peak_adc_{0};
  std::atomic<float> last_goodput_rx_{0.f};
  std::atomic<float> last_goodput_tx_{0.f};

  // Ping tracking
  struct PingState {
    std::mutex mu;
    std::atomic<bool> in_progress{false};
    float last_rtt = 0.f;
    float min_rtt = 0.f;
    float max_rtt = 0.f;
    float avg_rtt = 0.f;
    double sum_rtt = 0.0;
    uint32_t sent = 0;
    uint32_t rcvd = 0;
    std::string status = "Idle";
    std::chrono::steady_clock::time_point last_ping_time = std::chrono::steady_clock::now();
  } ping_;

  // Chat tracking
  std::mutex chat_mu_;
  std::vector<ChatMessage> chat_log_;
  std::vector<std::string> pending_chat_tx_;

  // Stage visualization snapshot
  std::mutex viz_mu_;
  FrameResult last_frame_viz_;
  std::vector<float> adc_spec_psd_;
  std::vector<c32> adc_iq_trace_;

  // Worker threads
  std::thread rx_th_;
  std::thread tx_th_;
  std::thread srv_th_;
  std::thread ping_th_;

  void rx_loop();
  void tx_loop();
  void srv_loop();
  void ping_loop();
  void run_single_ping();

  void on_client_command(const ClientMsg& cm);
  void broadcast_status();
  void broadcast_viz();
  void broadcast_chat();

  std::string detect_hostname();
  std::string derive_ip_cidr(const std::string& host);
};

}  // namespace quadrf_wifi
