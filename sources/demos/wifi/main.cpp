#include "wifi_constants.hpp"
#include "wifi_dsp.hpp"
#include "wifi_link.hpp"
#include "wifi_mac.hpp"
#include "wifi_tx.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace quadrf_wifi;

static std::atomic<bool> g_running{true};

static void sig_handler(int) {
  g_running = false;
}

int main(int argc, char* argv[]) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  WifiLink::Config cfg;
  cfg.port = 8092;
  cfg.freq_mhz = 5860.0;
  cfg.rate_str = "6";
  cfg.tx_gain = 34.0;
  cfg.rx_gain = 30.0;
  cfg.tx_elements = 0x01;
  cfg.rx_elements = 0x01;
  cfg.pa_mute = true;
  cfg.tun_name = "tun0";
  cfg.mtu = 576;

  bool smoke_test = false;
  bool launch_gui = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) cfg.port = std::stoi(argv[++i]);
    else if (arg == "--web-dir" && i + 1 < argc) cfg.web_dir = argv[++i];
    else if (arg == "--tun" && i + 1 < argc) cfg.tun_name = argv[++i];
    else if (arg == "--ip" && i + 1 < argc) cfg.ip_cidr = argv[++i];
    else if (arg == "--freq" && i + 1 < argc) cfg.freq_mhz = std::stod(argv[++i]);
    else if (arg == "--rate" && i + 1 < argc) cfg.rate_str = argv[++i];
    else if (arg == "--tx-gain" && i + 1 < argc) cfg.tx_gain = std::stod(argv[++i]);
    else if (arg == "--rx-gain" && i + 1 < argc) cfg.rx_gain = std::stod(argv[++i]);
    else if (arg == "--tx-ant" && i + 1 < argc) cfg.tx_elements = std::stoi(argv[++i], nullptr, 0);
    else if (arg == "--rx-ant" && i + 1 < argc) cfg.rx_elements = std::stoi(argv[++i], nullptr, 0);
    else if (arg == "--pa-mute") cfg.pa_mute = true;
    else if (arg == "--no-pa-mute") cfg.pa_mute = false;
    else if (arg == "--mtu" && i + 1 < argc) cfg.mtu = std::stoi(argv[++i]);
    else if (arg == "--smoke-test") smoke_test = true;
    else if (arg == "--gui") launch_gui = true;
    else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "  --port <port>           Web HUD / WebSocket port (default: 8092)\n"
                << "  --web-dir <path>        Static web assets path\n"
                << "  --tun <name>            Linux TUN interface name (default: tun0)\n"
                << "  --ip <cidr>             IP address with CIDR (default: auto from hostname)\n"
                << "  --freq <MHz>            Center carrier frequency (default: 5860.0)\n"
                << "  --rate <name>           Modulation rate: 6, 9, 12, 18, 24, 36, 48, 54\n"
                << "  --tx-gain <dB>          Transmit VGA gain in dB (0..25, default 20.0)\n"
                << "  --rx-gain <dB>          Receive baseband gain in dB (0..63, default 25.0)\n"
                << "  --tx-ant <mask>         Transmit active antenna mask (default: 0x01 = Element 0)\n"
                << "  --rx-ant <mask>         Receive active antenna mask (default: 0x01 = Element 0)\n"
                << "  --pa-mute               Mute PA during RX (Fast TDD, default)\n"
                << "  --no-pa-mute            Keep PA unmuted (Continuous TX)\n"
                << "  --smoke-test            Run offline synthetic DSP validation\n";
      return 0;
    }
  }

  cfg.tx_gain = std::min(40.0, std::max(0.0, cfg.tx_gain));
  cfg.rx_gain = std::min(63.0, std::max(0.0, cfg.rx_gain));

  if (launch_gui) {
    int rc = std::system(
        "sudo -n /usr/sbin/quadrf-app start data >/dev/null && "
        "resp=$(curl -sS -X POST 'http://127.0.0.1:8080/api/apps/open?app=data' 2>/dev/null || true); "
        "case \"$resp\" in *'\"opened\": true'*|*'\"opened\":true'*) exit 0 ;; esac; "
        "url='http://localhost/data/'; "
        "if command -v xdg-open >/dev/null; then exec xdg-open \"$url\"; fi; "
        "if command -v chromium >/dev/null; then exec chromium --app=\"$url\"; fi; "
        "exit 0");
    return (rc == 0) ? 0 : 1;
  }

  if (smoke_test) {
    std::cout << "Running offline synthetic DSP validation across all rates...\n";
    WifiDsp dsp;
    WifiTx tx;
    const std::vector<const RateInfo*> test_rates = {
        &kRate6, &kRate9, &kRate12, &kRate18, &kRate24, &kRate36, &kRate48, &kRate54};

    bool all_ok = true;
    for (const auto* r : test_rates) {
      std::string test_payload = "QuadRF IEEE 802.11 Loopback Test at " + std::string(r->name) + " Mbps";
      std::vector<uint8_t> ip_packet(test_payload.begin(), test_payload.end());
      auto mpdu = WifiMac::encapsulate_ipv4(ip_packet, 100);
      auto iq = tx.modulate_frame(mpdu, *r);
      FrameResult fr = dsp.decode_burst(iq);
      if (!fr.fcs_ok) {
        std::cerr << "FAIL: rate " << r->name << "\n";
        all_ok = false;
      } else {
        std::cout << "PASS: rate " << r->name << " (SNR " << fr.snr_db << " dB)\n";
      }
    }
    return all_ok ? 0 : 1;
  }

  std::cout << "=========================================================\n"
            << "  QuadRF 802.11 OFDM Link Transceiver & Web HUD\n"
            << "=========================================================\n"
            << "Port       : " << cfg.port << "\n"
            << "Carrier    : " << cfg.freq_mhz << " MHz\n"
            << "Rate       : " << cfg.rate_str << " Mbps\n"
            << "TX Gain    : " << cfg.tx_gain << " dB (Antenna 0x" << std::hex << cfg.tx_elements << std::dec << ")\n"
            << "RX Gain    : " << cfg.rx_gain << " dB (Antenna 0x" << std::hex << cfg.rx_elements << std::dec << ")\n"
            << "PA Control : " << (cfg.pa_mute ? "MUTE (Fast TDD)" : "UNMUTED (Zero Turnaround)") << "\n"
            << "---------------------------------------------------------\n";

  WifiLink link(cfg);
  std::string err_msg;
  if (!link.start(err_msg)) {
    std::cerr << "FATAL: " << err_msg << "\n";
    return 1;
  }

  while (g_running.load() && link.is_running()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\nShutting down 802.11 OFDM Link...\n";
  link.stop();
  return 0;
}
