#include "wifi_constants.hpp"
#include "wifi_dsp.hpp"
#include "wifi_gui.hpp"
#include "wifi_mac.hpp"
#include "wifi_radio.hpp"
#include "wifi_tun.hpp"
#include "wifi_tx.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iterator>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

using namespace quadrf_wifi;

static std::atomic<bool> g_running{true};
static std::atomic<int64_t> g_last_rx_ms{0};

static void sig_handler(int) {
  g_running = false;
}

// Desktop/service launch is `--gui` with no --ip. Both units used to take
// 10.99.0.1, so ICMP to 10.99.0.2 had no peer. Map QUADRF_HOSTNAME
// (quadrf → .1, quadrf-2 → .2) from /etc/quadrf/quadrf.conf.
static std::string default_tun_cidr() {
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
    if (line.compare(0, key.size(), key) != 0) continue;
    host = line.substr(key.size());
    auto last = host.find_last_not_of(" \t\r\n");
    if (last == std::string::npos) {
      host = "quadrf";
      continue;
    }
    host.erase(last + 1);
    if (host.size() >= 2 &&
        ((host.front() == '"' && host.back() == '"') ||
         (host.front() == '\'' && host.back() == '\''))) {
      host = host.substr(1, host.size() - 2);
    }
  }

  int node = 1;
  const auto dash = host.rfind('-');
  if (dash != std::string::npos) {
    try {
      node = std::stoi(host.substr(dash + 1));
    } catch (...) {
      node = 1;
    }
    if (node < 1 || node > 254) node = 1;
  }

  char buf[32];
  std::snprintf(buf, sizeof(buf), "10.99.0.%d/24", node);
  return buf;
}

int main(int argc, char* argv[]) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  std::string mode = "duplex";
  std::string tun_name = "tun0";
  std::string ip_cidr = default_tun_cidr();
  std::string mac_str = "";
  double freq_mhz = 5860.0;
  std::string rate_str = "6";
  double tx_gain = 0.0;
  double rx_gain = 50.0;
  std::string pol = "lhcp";
  int rx_ant = 0x01;
  int tx_ant = 0x01;
  bool enable_pa_mute = true;
  float cca_thresh = 0.00008f;
  bool use_gui = false;
  bool smoke_test = false;
  bool rf_loopback = false;
  bool verbose = false;
  int mtu = 1400;
  std::string dump_constellation_path = "";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--mode" && i + 1 < argc) mode = argv[++i];
    else if (arg == "--tun" && i + 1 < argc) tun_name = argv[++i];
    else if (arg == "--ip" && i + 1 < argc) ip_cidr = argv[++i];
    else if (arg == "--mac" && i + 1 < argc) mac_str = argv[++i];
    else if (arg == "--freq" && i + 1 < argc) freq_mhz = std::stod(argv[++i]);
    else if (arg == "--rate" && i + 1 < argc) rate_str = argv[++i];
    else if (arg == "--tx-gain" && i + 1 < argc) tx_gain = std::stod(argv[++i]);
    else if (arg == "--rx-gain" && i + 1 < argc) rx_gain = std::stod(argv[++i]);
    else if (arg == "--pol" && i + 1 < argc) pol = argv[++i];
    else if (arg == "--rx-ant" && i + 1 < argc) rx_ant = std::stoi(argv[++i], nullptr, 0);
    else if (arg == "--tx-ant" && i + 1 < argc) tx_ant = std::stoi(argv[++i], nullptr, 0);
    else if (arg == "--pa-mute") enable_pa_mute = true;
    else if (arg == "--no-pa-mute") enable_pa_mute = false;
    else if (arg == "--cca-thresh" && i + 1 < argc) cca_thresh = std::stof(argv[++i]);
    else if (arg == "--gui") use_gui = true;
    else if (arg == "--headless") use_gui = false;
    else if (arg == "--mtu" && i + 1 < argc) mtu = std::stoi(argv[++i]);
    else if (arg == "--smoke-test") smoke_test = true;
    else if (arg == "--rf-loopback") rf_loopback = true;
    else if (arg == "--dump-constellation" && i + 1 < argc) dump_constellation_path = argv[++i];
    else if (arg == "--verbose") verbose = true;
    else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "  --mode <duplex|tx|rx>   Operating mode (default: duplex)\n"
                << "  --tun <name>            TUN interface name (default: tun0)\n"
                << "  --ip <cidr>             IP address with CIDR (default: from QUADRF_HOSTNAME)\n"
                << "  --mac <xx:xx:...>       Node MAC address (default: derived from IP)\n"
                << "  --freq <MHz>            Center frequency in MHz (default: 5860.0)\n"
                << "  --rate <name>           Modulation rate (6, 9, 12, 18, 24, 36, 48, 54)\n"
                << "  --tx-gain <dB>          TX gain (0..25 dB, default 0, clamped <= 25)\n"
                << "  --rx-gain <dB>          RX gain (0..63 dB, recommended 20..30)\n"
                << "  --pol <lhcp|rhcp>       RX Polarization (default: lhcp)\n"
                << "  --rx-ant <mask>         RX antenna mask (default: 0x01)\n"
                << "  --tx-ant <mask>         TX antenna mask (default: 0x01)\n"
                << "  --pa-mute               Mute PA while listening (default)\n"
                << "  --no-pa-mute            Keep PA unmuted\n"
                << "  --gui                   Launch SDL dashboard\n"
                << "  --headless              Run without GUI\n"
                << "  --verbose               Per-frame RX/TX and 2 s telemetry\n"
                << "  --smoke-test            Run DSP self-test\n";
      return 0;
    }
  }

  tx_gain = std::min(25.0, std::max(0.0, tx_gain));

  // Node MAC: --mac, else 02:00 + IPv4 bytes (10.99.0.1 -> 02:00:0a:63:00:01)
  MacAddr node_mac = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  if (!mac_str.empty()) {
    node_mac = WifiMac::parse_mac(mac_str);
  } else {
    unsigned int ip_b[4];
    if (std::sscanf(ip_cidr.c_str(), "%u.%u.%u.%u", &ip_b[0], &ip_b[1], &ip_b[2], &ip_b[3]) == 4) {
      node_mac[0] = 0x02;
      node_mac[1] = 0x00;
      node_mac[2] = static_cast<uint8_t>(ip_b[0]);
      node_mac[3] = static_cast<uint8_t>(ip_b[1]);
      node_mac[4] = static_cast<uint8_t>(ip_b[2]);
      node_mac[5] = static_cast<uint8_t>(ip_b[3]);
    }
  }

  const RateInfo* initial_rate = rate_info_by_name(rate_str);
  if (!initial_rate) initial_rate = &kRate6;

  std::cout << "=========================================================\n"
            << "  802.11 Data Link\n"
            << "=========================================================\n"
            << "Mode       : " << mode << "\n"
            << "TUN Iface  : " << tun_name << " (" << ip_cidr << ", MTU " << mtu << ")\n"
            << "Node MAC   : " << WifiMac::format_mac(node_mac) << "\n"
            << "Frequency  : " << freq_mhz << " MHz\n"
            << "Initial Rate: " << initial_rate->name << "\n"
            << "TX Gain    : " << tx_gain << " dB (Enforced <= 25 dB, Ant 0x" << std::hex << tx_ant << std::dec << ")\n"
            << "RX Gain    : " << rx_gain << " dB (Ant 0x" << std::hex << rx_ant << std::dec << ", Pol " << pol << ")\n"
            << "PA Control : " << (enable_pa_mute ? "MUTE (Fast TDD Mute)" : "UNMUTED (Zero Turnaround)") << "\n"
            << "Interface  : " << (use_gui ? "SDL2 GUI" : "Headless Console") << "\n"
            << "---------------------------------------------------------\n";

  if (smoke_test) {
    std::cout << "Running offline synthetic DSP self-test across all rates...\n";
    WifiDsp dsp;
    WifiTx tx;
    const std::vector<const RateInfo*> test_rates = {
        &kRate6, &kRate9, &kRate12, &kRate18, &kRate24, &kRate36, &kRate48, &kRate54};

    bool all_ok = true;
    for (const auto* r : test_rates) {
      std::string test_payload = "QuadRF IEEE 802.11 TUN Loopback Payload Test at " + std::string(r->name);
      std::vector<uint8_t> ip_packet(test_payload.begin(), test_payload.end());
      auto mpdu = WifiMac::encapsulate_ipv4(ip_packet, 100);
      auto iq = tx.modulate_frame(mpdu, *r);
      FrameResult fr = dsp.decode_burst(iq);
      if (!fr.fcs_ok) {
        std::cerr << "FAIL: " << r->name << " loopback FCS mismatch\n";
        all_ok = false;
      } else {
        std::cout << "PASS: " << r->name << " (SNR: " << fr.snr_db << " dB)\n";
      }
    }
    return all_ok ? 0 : 1;
  }

  if (rf_loopback) {
    std::cout << "=========================================================\n"
              << "     Starting Live SDR Hardware RF Loopback Benchmark    \n"
              << "=========================================================\n";
    RadioConfig r_cfg;
    r_cfg.freq_mhz = freq_mhz;
    r_cfg.rx_gain = rx_gain;
    r_cfg.tx_gain = tx_gain;
    r_cfg.pol = pol;
    r_cfg.rx_antenna_mask = rx_ant;
    r_cfg.tx_antenna_mask = tx_ant;
    r_cfg.enable_pa_mute = false;  // Unmuted so RX receives TX transmission
    WifiRadio radio(r_cfg);
    if (!radio.init()) {
      std::cerr << "Fatal: Failed to initialize SDR radio\n";
      return 1;
    }

    std::cout << "Radio ready: TX Gain=" << tx_gain << " dB, RX Gain=" << rx_gain
              << " dB, Freq=" << freq_mhz << " MHz, Ant=" << tx_ant << "\n\n";

    WifiTx tx;
    DspConfig dsp_cfg;
    dsp_cfg.cca_threshold = cca_thresh;
    WifiDsp dsp(dsp_cfg);

    const std::vector<const RateInfo*> test_rates = {
        &kRate6, &kRate9, &kRate12, &kRate18, &kRate24, &kRate36};

    std::atomic<bool> rx_running{true};
    std::atomic<uint64_t> total_fcs_ok{0};
    std::atomic<uint64_t> total_fcs_fail{0};
    std::atomic<float> latest_snr{0.f};
    std::atomic<float> latest_evm{0.f};
    std::atomic<float> latest_cfo{0.f};
    std::vector<std::string> summary_rows;

    std::thread rx_worker([&] {
      constexpr size_t kChunkSize = 8192;
      std::vector<c32> rx_buf(kChunkSize);
      std::vector<c32> ring_buf;
      ring_buf.reserve(65536);

      while (rx_running && g_running) {
        int n = radio.read_rx(rx_buf.data(), kChunkSize, 20000);
        if (n <= 0) continue;
        ring_buf.insert(ring_buf.end(), rx_buf.begin(), rx_buf.begin() + n);

        while (ring_buf.size() >= 400) {
          StsResult sts = dsp.detect_sts(ring_buf.data(), ring_buf.size());
          if (sts.trig_idx < 0) {
            if (ring_buf.size() > 48) {
              ring_buf.erase(ring_buf.begin(), ring_buf.end() - 48);
            }
            break;
          }

          const size_t burst_start = (sts.trig_idx >= 32) ? static_cast<size_t>(sts.trig_idx - 32) : 0;
          if (burst_start + 400 > ring_buf.size()) break;

          std::span<const c32> burst_span(&ring_buf[burst_start], ring_buf.size() - burst_start);
          FrameResult fr = dsp.decode_burst(burst_span, &sts);

          if (fr.signal_ok && fr.needed_samples > 0 &&
              burst_start + static_cast<size_t>(fr.needed_samples) > ring_buf.size()) {
            break;
          }

          if (fr.signal_ok) {
            if (fr.fcs_ok) {
              total_fcs_ok.fetch_add(1, std::memory_order_relaxed);
              latest_snr.store(fr.snr_db, std::memory_order_relaxed);
              latest_evm.store(fr.evm_db, std::memory_order_relaxed);
              latest_cfo.store(fr.fine_cfo_hz, std::memory_order_relaxed);

              if (!dump_constellation_path.empty() && !fr.eq_symbols.empty()) {
                std::ofstream ofs(dump_constellation_path, std::ios::app);
                if (ofs.is_open()) {
                  ofs << "# FRAME rate=" << fr.rate_name << " snr=" << fr.snr_db
                      << " evm=" << fr.evm_db << " cfo=" << fr.fine_cfo_hz
                      << " n_syms=" << fr.eq_symbols.size() << "\n";
                  for (const auto& s : fr.eq_symbols) {
                    ofs << s.real() << " " << s.imag() << "\n";
                  }
                }
              }
            } else {
              total_fcs_fail.fetch_add(1, std::memory_order_relaxed);
            }

            const size_t burst_len = (fr.needed_samples > 0)
                                         ? static_cast<size_t>(fr.needed_samples)
                                         : (400 + static_cast<size_t>(fr.pkt_len * 8));
            size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
            ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
          } else {
            size_t consumed = std::min(burst_start + 400, ring_buf.size());
            ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
          }
        }
      }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << std::left << std::setw(24) << "Modulation Rate"
              << std::setw(8) << "Size"
              << std::setw(8) << "FCS-OK"
              << std::setw(10) << "Delivery"
              << std::setw(12) << "SNR"
              << std::setw(12) << "EVM"
              << "Status\n";
    std::cout << std::string(75, '-') << "\n";

    for (const auto* r : test_rates) {
      if (!g_running) break;
      for (int pkt_sz : {100, 1400}) {
        if (!g_running) break;
        std::vector<uint8_t> dummy_ip(static_cast<size_t>(pkt_sz - 38));
        for (size_t i = 0; i < dummy_ip.size(); ++i) dummy_ip[i] = static_cast<uint8_t>(i & 0xFF);
        auto mpdu = WifiMac::encapsulate_ipv4(dummy_ip, 1234);
        auto cs8 = tx.modulate_frame_cs8(mpdu, *r);

        const uint64_t ok_before = total_fcs_ok.load();
        const uint64_t fail_before = total_fcs_fail.load();
        constexpr int kPackets = 5;

        for (int p = 0; p < kPackets; ++p) {
          radio.write_tx_cs8(cs8.data(), cs8.size() / 2, 50000);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const uint64_t ok_after = total_fcs_ok.load();
        const uint64_t fail_after = total_fcs_fail.load();
        const uint64_t ok_diff = ok_after - ok_before;
        const uint64_t fail_diff = fail_after - fail_before;
        const float pass_rate = (ok_diff + fail_diff > 0) ? (100.f * ok_diff / (ok_diff + fail_diff)) : 0.f;

        std::ostringstream oss;
        oss << std::left << std::setw(24) << r->name
            << std::setw(8) << (std::to_string(pkt_sz) + " B")
            << std::setw(8) << (std::to_string(ok_diff) + "/" + std::to_string(kPackets))
            << std::setw(10) << (std::to_string(static_cast<int>(pass_rate)) + "%")
            << std::setw(12) << (std::to_string(latest_snr.load()).substr(0, 5) + " dB")
            << std::setw(12) << (std::to_string(latest_evm.load()).substr(0, 6) + " dB")
            << ((ok_diff >= 3) ? "PASS" : "FAIL");
        std::cout << oss.str() << "\n" << std::flush;
        summary_rows.push_back(oss.str());
      }
    }

    rx_running = false;
    radio.close();
    if (rx_worker.joinable()) rx_worker.join();

    std::cout << "===================================================\n";
    return 0;
  }

  WifiTun tun;
  if (!tun.open(tun_name)) {
    std::cerr << "Fatal: Failed to open TUN device " << tun_name << "\n";
    return 1;
  }
  tun.configure_interface(ip_cidr, mtu);
  tun.start();

  RadioConfig r_cfg;
  r_cfg.freq_mhz = freq_mhz;
  r_cfg.rx_gain = rx_gain;
  r_cfg.tx_gain = tx_gain;
  r_cfg.pol = pol;
  r_cfg.rx_antenna_mask = rx_ant;
  r_cfg.tx_antenna_mask = tx_ant;
  r_cfg.enable_pa_mute = enable_pa_mute;
  WifiRadio radio(r_cfg);

  if (!radio.init()) {
    std::cerr << "Fatal: Failed to initialize SDR radio\n";
    return 1;
  }

  std::atomic<const RateInfo*> current_rate{initial_rate};
  std::atomic<bool> tx_radiating{false};
  std::atomic<int64_t> tx_finished_ms{0};
  std::atomic<bool> tunnel_active{true};

  struct TelemetryCounters {
    std::atomic<uint64_t> tx_packets{0};
    std::atomic<uint64_t> sts_triggers{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> fcs_ok{0};
    std::atomic<uint64_t> fcs_fail{0};
    std::atomic<float> last_snr{0.f};
    std::atomic<float> last_evm{0.f};
    std::atomic<float> last_cfo{0.f};
    std::atomic<uint64_t> rx_bytes{0};
    std::atomic<uint64_t> tx_bytes{0};
  } counters;

  struct PingManager {
    std::mutex mu;
    std::atomic<bool> in_progress{false};
    std::atomic<bool> auto_ping{false};
    std::string target_ip = "10.99.0.2";
    std::string status = "Idle - Click [PING PEER] to test link";
    float last_rtt_ms = 0.f;
    float min_rtt_ms = 0.f;
    float max_rtt_ms = 0.f;
    float avg_rtt_ms = 0.f;
    uint32_t sent = 0;
    uint32_t rcvd = 0;
    double sum_rtt_ms = 0.0;
    std::chrono::steady_clock::time_point last_ping_time = std::chrono::steady_clock::now();

    void trigger_ping() {
      bool expected = false;
      if (!in_progress.compare_exchange_strong(expected, true)) {
        return;
      }
      std::thread([this]() {
        run_ping_worker();
      }).detach();
    }

    void run_ping_worker() {
      std::string ip;
      {
        std::lock_guard<std::mutex> lock(mu);
        ip = target_ip;
        status = "Pinging " + ip + "...";
      }

      // Air RTT is ~60 ms; 400 ms covers TDD quiet + DSP jitter without a 2 s GUI stall
      std::string cmd = "ping -c 1 -W 0.4 " + ip + " 2>&1";
      FILE* pipe = popen(cmd.c_str(), "r");
      bool success = false;
      float rtt = 0.f;
      if (pipe) {
        char buf[256];
        while (fgets(buf, sizeof(buf), pipe)) {
          char* time_str = strstr(buf, "time=");
          if (time_str) {
            float val = 0.f;
            if (sscanf(time_str + 5, "%f", &val) == 1) {
              rtt = val;
              success = true;
            }
          }
        }
        int ret = pclose(pipe);
        if (ret != 0) {
          success = false;
        }
      }

      {
        std::lock_guard<std::mutex> lock(mu);
        sent++;
        if (success) {
          rcvd++;
          last_rtt_ms = rtt;
          sum_rtt_ms += rtt;
          if (min_rtt_ms == 0.f || rtt < min_rtt_ms) min_rtt_ms = rtt;
          if (rtt > max_rtt_ms) max_rtt_ms = rtt;
          avg_rtt_ms = static_cast<float>(sum_rtt_ms / rcvd);
          char sbuf[128];
          std::snprintf(sbuf, sizeof(sbuf), "Ping OK: %.1f ms (avg: %.1f ms)", rtt, avg_rtt_ms);
          status = sbuf;
        } else {
          status = "Ping TIMEOUT / Request timed out";
        }
      }
      in_progress.store(false, std::memory_order_release);
    }
  } ping_mgr;

  if (ip_cidr.find("10.99.0.1") != std::string::npos) {
    ping_mgr.target_ip = "10.99.0.2";
  } else {
    ping_mgr.target_ip = "10.99.0.1";
  }

  std::unique_ptr<WifiGui> gui;
  if (use_gui) {
    gui = std::make_unique<WifiGui>();
    if (!gui->init()) {
      std::cerr << "Warning: Could not open SDL2 window, falling back to headless mode\n";
      gui.reset();
    }
  }

  std::thread rx_thread;
  if (mode != "tx") {
    rx_thread = std::thread([&] {
    DspConfig dsp_cfg;
    dsp_cfg.cca_threshold = cca_thresh;
    WifiDsp dsp(dsp_cfg);
    constexpr size_t kChunkSize = 8192;
    std::vector<c32> rx_buf(kChunkSize);
    std::vector<c32> ring_buf;
    ring_buf.reserve(65536);

    MacRxConfig rx_mac_cfg;
    rx_mac_cfg.my_mac = node_mac;
    rx_mac_cfg.drop_self_packets = true;

    while (g_running) {
      int n = radio.read_rx(rx_buf.data(), kChunkSize, 50000);
      if (n <= 0) continue;

      // Blank RX during local transmission and for 5 ms settling time to prevent self-TX saturation
      const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      const int64_t last_tx = tx_finished_ms.load(std::memory_order_acquire);
      if (tx_radiating.load(std::memory_order_acquire) || (now_ms - last_tx >= 0 && now_ms - last_tx < 5)) {
        ring_buf.clear();
        continue;
      }

      ring_buf.insert(ring_buf.end(), rx_buf.begin(), rx_buf.begin() + n);

      while (ring_buf.size() >= 400) {
        StsResult sts = dsp.detect_sts(ring_buf.data(), ring_buf.size());
        if (sts.trig_idx < 0) {
          // Keep last 48 samples for overlapping sliding window
          if (ring_buf.size() > 48) {
            ring_buf.erase(ring_buf.begin(), ring_buf.end() - 48);
          }
          break;
        }

        counters.sts_triggers.fetch_add(1, std::memory_order_relaxed);
        // Back up 32 samples so full STS is included in burst_span
        const size_t burst_start = (sts.trig_idx >= 32) ? static_cast<size_t>(sts.trig_idx - 32) : 0;
        if (burst_start + 400 > ring_buf.size()) {
          // Need more samples to decode preamble + SIGNAL
          break;
        }

        std::span<const c32> burst_span(&ring_buf[burst_start], ring_buf.size() - burst_start);
        FrameResult fr = dsp.decode_burst(burst_span, &sts);

        // If SIGNAL was decoded but the full payload hasn't arrived yet, wait!
        if (fr.signal_ok && fr.needed_samples > 0 &&
            burst_start + static_cast<size_t>(fr.needed_samples) > ring_buf.size()) {
          break;
        }

        if (fr.signal_ok) {
          counters.rx_frames.fetch_add(1, std::memory_order_relaxed);
          if (fr.fcs_ok) {
            counters.fcs_ok.fetch_add(1, std::memory_order_relaxed);

            // Addr2 is our SA. Full-duplex RX still decodes our own frames
            // after the TX blank — Soapy's FIFO outlives that window.
            bool from_self = false;
            if (fr.payload.size() >= 16) {
              from_self = true;
              for (int i = 0; i < 6; ++i) {
                if (fr.payload[10 + i] != node_mac[static_cast<size_t>(i)]) {
                  from_self = false;
                  break;
                }
              }
            }

            if (!from_self) {
              counters.last_snr.store(fr.snr_db, std::memory_order_relaxed);
              counters.last_evm.store(fr.evm_db, std::memory_order_relaxed);
              counters.last_cfo.store(fr.fine_cfo_hz, std::memory_order_relaxed);
              if (gui && !fr.eq_symbols.empty()) {
                gui->add_constellation_points(fr.eq_symbols);
              }
            }

            // Extract sequence number from 802.11 MAC header (bytes 22..23)
            uint16_t seq_num = 0;
            if (fr.payload.size() >= 24) {
              const uint16_t seq_ctrl = static_cast<uint16_t>(fr.payload[22]) |
                                       (static_cast<uint16_t>(fr.payload[23]) << 8);
              seq_num = seq_ctrl >> 4;
            }

            static uint16_t s_last_seq = 0xFFFF;
            static auto s_last_rx_time = std::chrono::steady_clock::now();
            rx_mac_cfg.my_mac = node_mac;
            auto ip_pkt = WifiMac::decapsulate_ipv4(fr.payload, rx_mac_cfg);
            if (!ip_pkt.empty()) {
              auto now = std::chrono::steady_clock::now();
              double elapsed_ms = std::chrono::duration<double, std::milli>(now - s_last_rx_time).count();

              if (seq_num == s_last_seq && elapsed_ms < 500) {
                // Duplicate frame: silently suppress
              } else {
                s_last_seq = seq_num;
                s_last_rx_time = now;
                g_last_rx_ms.store(
                    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count(),
                    std::memory_order_release);
                if (tunnel_active.load(std::memory_order_relaxed)) {
                  tun.inject_packet(ip_pkt.data(), ip_pkt.size());
                }
                if (verbose) {
                  std::cout << "[RX] Decoded & Injected " << ip_pkt.size() << " B into TUN ("
                            << fr.rate_name << ", SNR: " << fr.snr_db << " dB, EVM: " << fr.evm_db
                            << " dB, ltf1: " << fr.ltf1_idx << " (peak " << fr.ltf_peak
                            << "), CFO: " << fr.fine_cfo_hz << " Hz, seq: " << seq_num << ")"
                            << (tunnel_active.load() ? "" : " [TUNNEL INACTIVE]") << "\n" << std::flush;
                }

                if (!dump_constellation_path.empty() && !fr.eq_symbols.empty()) {
                  std::ofstream ofs(dump_constellation_path, std::ios::app);
                  if (ofs.is_open()) {
                    ofs << "# FRAME rate=" << fr.rate_name << " snr=" << fr.snr_db << " evm=" << fr.evm_db
                        << " cfo=" << fr.fine_cfo_hz << " n_syms=" << fr.eq_symbols.size()
                        << " fcs=" << (fr.fcs_ok ? "OK" : "FAIL") << "\n";
                    for (const auto& s : fr.eq_symbols) {
                      ofs << s.real() << " " << s.imag() << "\n";
                    }
                  }
                }
              }
            }

            const size_t burst_len = (fr.needed_samples > 0)
                                         ? static_cast<size_t>(fr.needed_samples)
                                         : (400 + static_cast<size_t>(fr.pkt_len * 8));
            size_t consumed = burst_start + burst_len;
            consumed = std::min(consumed, ring_buf.size());
            ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
          } else {
            counters.fcs_fail.fetch_add(1, std::memory_order_relaxed);
            if (verbose) {
              std::ostringstream hex_oss;
              for (size_t i = 0; i < std::min<size_t>(128, fr.payload.size()); ++i) {
                hex_oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(fr.payload[i]) << " ";
              }
              std::cout << "[RX-FAIL] FCS error (" << fr.rate_name
                        << ", len: " << fr.pkt_len << " B, SNR: " << fr.snr_db
                        << " dB, EVM: " << fr.evm_db
                        << " dB, ltf1: " << fr.ltf1_idx << " (peak " << fr.ltf_peak
                        << "), coarse CFO: " << fr.coarse_cfo_hz
                        << " Hz, fine CFO: " << fr.fine_cfo_hz
                        << " Hz, resid CFO: " << fr.resid_cfo_hz
                        << " Hz)\n  Hex: " << hex_oss.str() << "\n" << std::flush;
            }

            if (!dump_constellation_path.empty() && !fr.eq_symbols.empty()) {
              std::ofstream ofs(dump_constellation_path, std::ios::app);
              if (ofs.is_open()) {
                ofs << "# FRAME rate=" << fr.rate_name << " snr=" << fr.snr_db << " evm=" << fr.evm_db
                    << " cfo=" << fr.fine_cfo_hz << " n_syms=" << fr.eq_symbols.size()
                    << " fcs=FAIL\n";
                for (const auto& s : fr.eq_symbols) {
                  ofs << s.real() << " " << s.imag() << "\n";
                }
              }
            }
            const size_t burst_len = (fr.needed_samples > 0)
                                         ? static_cast<size_t>(fr.needed_samples)
                                         : 400;
            size_t consumed = std::min(burst_start + burst_len, ring_buf.size());
            ring_buf.erase(ring_buf.begin(), ring_buf.begin() + consumed);
          }
        } else {
          static int drop_count = 0;
          ++drop_count;
          if (verbose && (fr.ltf_peak >= 0.12f || (drop_count % 2000 == 0))) {
            std::cout << "[RX-DROP #" << drop_count << "] STS metric " << sts.metric_at_trig << " at idx " << sts.trig_idx
                      << " (pwr: " << sts.R_at_trig << ", ltf_peak: " << fr.ltf_peak << ")\n" << std::flush;
          }
          const size_t skip_samples = (sts.R_at_trig > 0.05f) ? 2000 : 200;
          const size_t advance = std::min<size_t>(static_cast<size_t>(sts.trig_idx) + skip_samples, ring_buf.size());
          ring_buf.erase(ring_buf.begin(), ring_buf.begin() + advance);
        }
      }
    }
    });
  }

  std::thread tx_thread;
  if (mode != "rx") {
    tx_thread = std::thread([&] {
    WifiTx tx;
    uint16_t seq_num = 0;
    MacTxConfig tx_mac_cfg;

    while (g_running) {
      std::vector<uint8_t> ip_packet;
      if (!tun.tx_queue().pop(ip_packet, 50)) continue;
      if (ip_packet.empty()) continue;
      if (!tunnel_active.load(std::memory_order_relaxed)) continue;

      tx_mac_cfg.sa = node_mac;
      auto mpdu = WifiMac::encapsulate_ipv4(ip_packet, seq_num++, tx_mac_cfg);

      const RateInfo* r = current_rate.load(std::memory_order_relaxed);
      auto cs8 = tx.modulate_frame_cs8(mpdu, *r);
      if (cs8.empty()) continue;

      int retries = 0;
      while (!radio.is_channel_clear() && retries++ < 20 && g_running) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }

      const int64_t last_rx = g_last_rx_ms.load(std::memory_order_acquire);
      if (last_rx > 0) {
        const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t elapsed = now_ms - last_rx;
        if (elapsed >= 0 && elapsed < 24) {
          std::this_thread::sleep_for(std::chrono::milliseconds(24 - elapsed));
        }
      }

      tx_radiating.store(true, std::memory_order_release);
      radio.write_tx_cs8(cs8.data(), cs8.size() / 2, 50000);
      tx_finished_ms.store(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count(),
          std::memory_order_release);
      tx_radiating.store(false, std::memory_order_release);

      counters.tx_packets.fetch_add(1, std::memory_order_relaxed);
      counters.tx_bytes.fetch_add(ip_packet.size(), std::memory_order_relaxed);
      if (verbose) {
        std::cout << "[TX] Radiated " << ip_packet.size() << " B (" << r->name
                  << ", " << cs8.size() / 2 << " samples)\n" << std::flush;
      }
    }
    });
  }

  if (gui) {
    gui->on_toggle_tunnel([&] {
      bool active = !tunnel_active.load();
      tunnel_active.store(active);
      std::cout << "[TUNNEL] Tunnel is now " << (active ? "ACTIVE" : "INACTIVE") << "\n" << std::flush;
    });

    gui->on_switch_preset([&] {
      if (ip_cidr.find("10.99.0.1") != std::string::npos) {
        ip_cidr = "10.99.0.2/24";
        node_mac = {0x02, 0x00, 0x0a, 0x63, 0x00, 0x02};
        {
          std::lock_guard<std::mutex> lock(ping_mgr.mu);
          ping_mgr.target_ip = "10.99.0.1";
          ping_mgr.status = "Target switched to 10.99.0.1";
        }
      } else {
        ip_cidr = "10.99.0.1/24";
        node_mac = {0x02, 0x00, 0x0a, 0x63, 0x00, 0x01};
        {
          std::lock_guard<std::mutex> lock(ping_mgr.mu);
          ping_mgr.target_ip = "10.99.0.2";
          ping_mgr.status = "Target switched to 10.99.0.2";
        }
      }
      tun.configure_interface(ip_cidr, mtu);
      std::cout << "[ROLE] Switched to preset: " << ip_cidr << " (MAC "
                << WifiMac::format_mac(node_mac) << ")\n" << std::flush;
    });

    gui->on_send_ping([&] {
      ping_mgr.trigger_ping();
    });

    gui->on_toggle_auto_ping([&] {
      bool cur = ping_mgr.auto_ping.load();
      ping_mgr.auto_ping.store(!cur);
      std::cout << "[PING] Auto-ping " << (!cur ? "ENABLED" : "DISABLED") << "\n" << std::flush;
    });

    gui->on_change_rate([&](int dir) {
      const int n = static_cast<int>(std::size(kGuiRates));
      auto it = std::find(std::begin(kGuiRates), std::end(kGuiRates), current_rate.load());
      int idx = (it != std::end(kGuiRates))
                    ? static_cast<int>(std::distance(std::begin(kGuiRates), it))
                    : 0;
      idx = (idx + dir + n) % n;
      current_rate.store(kGuiRates[static_cast<size_t>(idx)]);
      std::cout << "Rate switched to: " << kGuiRates[static_cast<size_t>(idx)]->name << "\n" << std::flush;
    });

  }

  auto last_print = std::chrono::steady_clock::now();
  uint64_t last_rx_bytes = 0;
  auto last_tput_time = std::chrono::steady_clock::now();
  double current_throughput_kbps = 0.0;

  while (g_running) {
    auto now = std::chrono::steady_clock::now();
    double tput_elapsed = std::chrono::duration<double>(now - last_tput_time).count();
    if (tput_elapsed >= 0.5) {
      uint64_t cur_bytes = counters.rx_bytes.load(std::memory_order_relaxed);
      current_throughput_kbps = (cur_bytes - last_rx_bytes) * 8.0 / (tput_elapsed * 1000.0);
      last_rx_bytes = cur_bytes;
      last_tput_time = now;
    }

    if (ping_mgr.auto_ping.load()) {
      auto now_p = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now_p - ping_mgr.last_ping_time).count() >= 1.5) {
        ping_mgr.last_ping_time = now_p;
        ping_mgr.trigger_ping();
      }
    }

    if (gui) {
      GuiTelemetry telem;
      telem.rate_name = current_rate.load()->name;
      telem.tunnel_up = tunnel_active.load(std::memory_order_relaxed);
      telem.tun_name = tun_name;
      telem.ip_cidr = ip_cidr;
      telem.mac_str = WifiMac::format_mac(node_mac);
      telem.tx_packets = counters.tx_packets.load(std::memory_order_relaxed);
      telem.rx_frames = counters.rx_frames.load(std::memory_order_relaxed);
      telem.fcs_ok = counters.fcs_ok.load(std::memory_order_relaxed);
      telem.fcs_fail = counters.fcs_fail.load(std::memory_order_relaxed);
      telem.last_snr_db = counters.last_snr.load(std::memory_order_relaxed);
      telem.last_evm_db = counters.last_evm.load(std::memory_order_relaxed);
      telem.last_cfo_hz = counters.last_cfo.load(std::memory_order_relaxed);
      telem.last_energy = radio.get_current_energy();
      telem.throughput_kbps = current_throughput_kbps;

      {
        std::lock_guard<std::mutex> lock(ping_mgr.mu);
        telem.ping_target = ping_mgr.target_ip;
        telem.ping_status = ping_mgr.status;
        telem.ping_rtt_ms = ping_mgr.last_rtt_ms;
        telem.ping_min_rtt_ms = ping_mgr.min_rtt_ms;
        telem.ping_max_rtt_ms = ping_mgr.max_rtt_ms;
        telem.ping_avg_rtt_ms = ping_mgr.avg_rtt_ms;
        telem.ping_sent = ping_mgr.sent;
        telem.ping_rcvd = ping_mgr.rcvd;
        telem.ping_in_progress = ping_mgr.in_progress.load();
        telem.auto_ping = ping_mgr.auto_ping.load();
      }

      gui->update_telemetry(telem);
      if (!gui->render_frame()) {
        g_running = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (verbose && std::chrono::duration<double>(now - last_print).count() >= 2.0) {
      last_print = now;
      const uint64_t tx = counters.tx_packets.load(std::memory_order_relaxed);
      const uint64_t rx = counters.rx_frames.load(std::memory_order_relaxed);
      const uint64_t ok = counters.fcs_ok.load(std::memory_order_relaxed);
      const uint64_t fail = counters.fcs_fail.load(std::memory_order_relaxed);
      const float p_rate = (ok + fail > 0) ? (100.f * ok / (ok + fail)) : 0.f;
      const uint64_t sts_trig = counters.sts_triggers.load(std::memory_order_relaxed);
      std::cout << "[TELEMETRY] " << (tunnel_active.load() ? "ACTIVE" : "INACTIVE")
                << " | TX: " << tx << " pkts | STS: " << sts_trig << " | SIG: " << rx
                << " | FCS OK: " << ok << " (" << p_rate << "%) | SNR: " << counters.last_snr.load()
                << " dB | CFO: " << counters.last_cfo.load() << " Hz | Energy: "
                << radio.get_current_energy() << " | Tput: " << current_throughput_kbps << " kbps\n" << std::flush;
    }
  }

  std::cout << "\nShutting down...\n";
  g_running = false;
  tun.stop();
  radio.close();

  if (rx_thread.joinable()) rx_thread.join();
  if (tx_thread.joinable()) tx_thread.join();
  if (gui) gui->close();

  std::cout << "Clean shutdown complete.\n";
  return 0;
}
