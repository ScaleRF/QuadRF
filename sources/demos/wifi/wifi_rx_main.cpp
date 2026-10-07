// wifi_rx_main.cpp - Standalone IEEE 802.11a/g/n OFDM Wi-Fi Decoder for QuadRF.
//
// Demodulates live 20 MHz 802.11 channels (BPSK 6/9M, QPSK 12/18M, 16-QAM 24/36M, 64-QAM 48/54M)
// and extracts complete 802.11 MAC frames, SSIDs, BSSIDs, sequence numbers, LLC/SNAP, and IPv4 packets.
//
// Outputs JSON lines to stdout for PhaseGaze integration or rich terminal telemetry for engineers.

#include "wifi_constants.hpp"
#include "wifi_dsp.hpp"
#include "wifi_mac.hpp"
#include "wifi_radio.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

using namespace quadrf_wifi;

static std::atomic<bool> g_running{true};

static void sig_handler(int) {
  g_running = false;
}

struct WifiChannel {
  int ch;
  double freq_mhz;
  const char* band;
};

static const WifiChannel kKnownChannels[] = {
    // 5 GHz UNII-1
    {36, 5180.0, "5 GHz"}, {40, 5200.0, "5 GHz"}, {44, 5220.0, "5 GHz"}, {48, 5240.0, "5 GHz"},
    // 5 GHz UNII-2A
    {52, 5260.0, "5 GHz"}, {56, 5280.0, "5 GHz"}, {60, 5300.0, "5 GHz"}, {64, 5320.0, "5 GHz"},
    // 5 GHz UNII-2C
    {100, 5500.0, "5 GHz"}, {104, 5520.0, "5 GHz"}, {108, 5540.0, "5 GHz"}, {112, 5560.0, "5 GHz"},
    {116, 5580.0, "5 GHz"}, {120, 5600.0, "5 GHz"}, {124, 5620.0, "5 GHz"}, {128, 5640.0, "5 GHz"},
    {132, 5660.0, "5 GHz"}, {136, 5680.0, "5 GHz"}, {140, 5700.0, "5 GHz"}, {144, 5720.0, "5 GHz"},
    // 5 GHz UNII-3 / UNII-4
    {149, 5745.0, "5 GHz"}, {153, 5765.0, "5 GHz"}, {157, 5785.0, "5 GHz"}, {161, 5805.0, "5 GHz"},
    {165, 5825.0, "5 GHz"}, {169, 5845.0, "5 GHz"}, {173, 5865.0, "5 GHz"}, {177, 5885.0, "5 GHz"},
    // 6 GHz (802.11ax / 6E 20 MHz channels)
    {1, 5955.0, "6 GHz"}, {5, 5975.0, "6 GHz"}, {9, 5995.0, "6 GHz"}, {13, 6015.0, "6 GHz"},
    {17, 6035.0, "6 GHz"}, {21, 6055.0, "6 GHz"}, {25, 6075.0, "6 GHz"}, {29, 6095.0, "6 GHz"},
    {33, 6115.0, "6 GHz"}, {37, 6135.0, "6 GHz"}, {41, 6155.0, "6 GHz"}, {45, 6175.0, "6 GHz"},
    {49, 6195.0, "6 GHz"}, {53, 6215.0, "6 GHz"}, {57, 6235.0, "6 GHz"}, {61, 6255.0, "6 GHz"},
    {65, 6275.0, "6 GHz"}, {69, 6295.0, "6 GHz"}, {73, 6315.0, "6 GHz"}, {77, 6335.0, "6 GHz"},
    {81, 6355.0, "6 GHz"}, {85, 6375.0, "6 GHz"}, {89, 6395.0, "6 GHz"}, {93, 6415.0, "6 GHz"},
    {97, 6435.0, "6 GHz"}, {101, 6455.0, "6 GHz"}, {105, 6475.0, "6 GHz"}, {109, 6495.0, "6 GHz"}
};

static const WifiChannel* snap_channel(double mhz) {
  const WifiChannel* best = nullptr;
  double min_diff = 1e9;
  for (const auto& c : kKnownChannels) {
    const double diff = std::abs(c.freq_mhz - mhz);
    if (diff < min_diff) {
      min_diff = diff;
      best = &c;
    }
  }
  return (best && min_diff <= 12.0) ? best : nullptr;
}

static const WifiChannel* find_channel_by_num(int ch) {
  for (const auto& c : kKnownChannels) {
    if (c.ch == ch) return &c;
  }
  return nullptr;
}

int main(int argc, char* argv[]) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);  // Line buffered for streaming JSON pipes
  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  double freq_mhz = 5180.0;  // Default Channel 36
  int channel_num = 0;
  double rx_gain = 55.0;     // Optimal sensitivity for QuadRF phased array summing
  int rx_ant = 0x0F;         // All 4 antennas summed for maximum phased array aperture gain
  std::string pol = "rhcp";
  float cca_thresh = 0.00018f;
  float sts_thresh = 0.42f;
  bool fcs_only = false;
  bool stdout_json = false;
  bool pretty = true;
  bool include_constellation = false;
  bool skip_lo_retune = false;
  std::string raw_file_in = "";
  std::string raw_file_out = "";
  int max_packets = 0;
  int timeout_sec = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--skip-lo-retune") {
      skip_lo_retune = true;
    } else if ((arg == "--freq" || arg == "-f") && i + 1 < argc) {
      freq_mhz = std::stod(argv[++i]);
    } else if ((arg == "--channel" || arg == "-c") && i + 1 < argc) {
      channel_num = std::stoi(argv[++i]);
    } else if (arg == "--rx-gain" && i + 1 < argc) {
      rx_gain = std::stod(argv[++i]);
    } else if (arg == "--rx-ant" && i + 1 < argc) {
      rx_ant = std::stoi(argv[++i], nullptr, 0);
    } else if (arg == "--pol" && i + 1 < argc) {
      pol = argv[++i];
    } else if (arg == "--cca-thresh" && i + 1 < argc) {
      cca_thresh = std::stof(argv[++i]);
    } else if (arg == "--sts-thresh" && i + 1 < argc) {
      sts_thresh = std::stof(argv[++i]);
    } else if (arg == "--fcs-only") {
      fcs_only = true;
    } else if (arg == "--stdout-json" || arg == "--json") {
      stdout_json = true;
      pretty = false;
    } else if (arg == "--pretty") {
      pretty = true;
      stdout_json = false;
    } else if (arg == "--constellation") {
      include_constellation = true;
    } else if (arg == "--raw-file" && i + 1 < argc) {
      raw_file_in = argv[++i];
    } else if (arg == "--dump-raw" && i + 1 < argc) {
      raw_file_out = argv[++i];
    } else if (arg == "--max-packets" && i + 1 < argc) {
      max_packets = std::stoi(argv[++i]);
    } else if (arg == "--timeout" && i + 1 < argc) {
      timeout_sec = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "  --channel, -c <num>     Wi-Fi Channel (36, 40, 44, 48, 149, 157, 69, ...)\n"
                << "  --freq, -f <MHz>        Center frequency in MHz (default: 5180.0)\n"
                << "  --rx-gain <dB>          RX gain (0..63 dB, default: 45.0)\n"
                << "  --rx-ant <mask>         RX antenna mask (default: 0x0F = all 4 antennas)\n"
                << "  --pol <rhcp|lhcp>       Polarization (default: rhcp)\n"
                << "  --stdout-json, --json   Stream one JSON line per packet (for PhaseGaze / IPC)\n"
                << "  --constellation         Include complex constellation points in JSON\n"
                << "  --raw-file <path.cs8>   Replay from pre-recorded CS8 file\n"
                << "  --dump-raw <path.cs8>   Tap and record raw CS8 stream to file\n"
                << "  --max-packets <N>       Exit after receiving N packets\n"
                << "  --timeout <seconds>     Exit after specified runtime\n";
      return 0;
    }
  }

  // Resolve channel
  const WifiChannel* cur_ch = nullptr;
  if (channel_num > 0) {
    cur_ch = find_channel_by_num(channel_num);
    if (cur_ch) {
      freq_mhz = cur_ch->freq_mhz;
    } else {
      std::cerr << "Warning: Channel " << channel_num << " not in standard table, using specified frequency " << freq_mhz << " MHz\n";
    }
  } else {
    cur_ch = snap_channel(freq_mhz);
    if (cur_ch) {
      freq_mhz = cur_ch->freq_mhz;
      channel_num = cur_ch->ch;
    }
  }

  if (pretty) {
    std::cout << "=========================================================\n"
              << "       QuadRF Live IEEE 802.11 Wi-Fi Decoder             \n"
              << "=========================================================\n"
              << "Frequency  : " << freq_mhz << " MHz";
    if (cur_ch) std::cout << " (Channel " << cur_ch->ch << ", " << cur_ch->band << ")";
    std::cout << "\nBandwidth  : 20 MHz (64-subcarrier OFDM @ 20 MS/s)\n"
              << "RX Gain    : " << rx_gain << " dB\n"
              << "Antennas   : 0x" << std::hex << rx_ant << std::dec
              << " (" << ((rx_ant == 0x0F) ? "all 4 summed" : "single element") << ")\n"
              << "Polarization: " << pol << "\n";
    if (!raw_file_in.empty()) {
      std::cout << "Source     : File " << raw_file_in << "\n";
    } else {
      std::cout << "Source     : Live SoapySDR (driver=mipi)\n";
    }
    std::cout << "---------------------------------------------------------\n";
  }

  DspConfig dsp_cfg;
  dsp_cfg.cca_threshold = cca_thresh;
  dsp_cfg.sts_threshold = sts_thresh;
  dsp_cfg.sts_consecutive = 3;
  dsp_cfg.enable_cir_smoothing = false;
  WifiDsp dsp(dsp_cfg);

  std::unique_ptr<WifiRadio> radio;
  std::ifstream raw_in;
  std::ofstream raw_out;

  if (!raw_file_in.empty()) {
    raw_in.open(raw_file_in, std::ios::binary);
    if (!raw_in.is_open()) {
      std::cerr << "[-] Error opening raw input file: " << raw_file_in << "\n";
      return 1;
    }
  } else {
    RadioConfig radio_cfg;
    radio_cfg.freq_mhz = freq_mhz;
    radio_cfg.rx_gain = rx_gain;
    radio_cfg.rx_antenna_mask = rx_ant;
    radio_cfg.pol = pol;
    radio_cfg.enable_pa_mute = true;
    radio_cfg.skip_lo_retune = skip_lo_retune;
    radio_cfg.cca_threshold = cca_thresh;

    radio = std::make_unique<WifiRadio>(radio_cfg);
    if (!radio->init()) {
      std::cerr << "[-] Error initializing QuadRF radio\n";
      return 1;
    }
  }

  if (!raw_file_out.empty()) {
    raw_out.open(raw_file_out, std::ios::binary);
    if (!raw_out.is_open()) {
      std::cerr << "[-] Warning: cannot open dump output file: " << raw_file_out << "\n";
    }
  }

  constexpr size_t kChunk = 8192;
  std::vector<c32> chunk_buf(kChunk);
  std::vector<int8_t> cs8_buf(kChunk * 2);
  std::vector<c32> ring_buf;
  ring_buf.reserve(65536);

  uint64_t total_packets = 0;
  uint64_t valid_fcs_count = 0;
  auto t_start = std::chrono::steady_clock::now();

  while (g_running) {
    if (timeout_sec > 0) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - t_start).count();
      if (elapsed >= timeout_sec) break;
    }

    size_t got_samples = 0;
    if (radio) {
      int n = radio->read_rx(chunk_buf.data(), kChunk, 50000);
      if (n <= 0) continue;
      got_samples = static_cast<size_t>(n);

      if (raw_out.is_open()) {
        for (size_t i = 0; i < got_samples; ++i) {
          int8_t r = static_cast<int8_t>(std::clamp(chunk_buf[i].real() * 127.f, -128.f, 127.f));
          int8_t im = static_cast<int8_t>(std::clamp(chunk_buf[i].imag() * 127.f, -128.f, 127.f));
          cs8_buf[2 * i + 0] = r;
          cs8_buf[2 * i + 1] = im;
        }
        raw_out.write(reinterpret_cast<const char*>(cs8_buf.data()), got_samples * 2);
      }
    } else {
      raw_in.read(reinterpret_cast<char*>(cs8_buf.data()), kChunk * 2);
      std::streamsize bytes_read = raw_in.gcount();
      if (bytes_read <= 0) break;
      got_samples = static_cast<size_t>(bytes_read / 2);
      for (size_t i = 0; i < got_samples; ++i) {
        chunk_buf[i] = c32(static_cast<float>(cs8_buf[2 * i + 0]) / 128.f,
                           static_cast<float>(cs8_buf[2 * i + 1]) / 128.f);
      }
    }

    ring_buf.insert(ring_buf.end(), chunk_buf.begin(), chunk_buf.begin() + got_samples);

    // Slide across buffer and detect burst triggers
    while (ring_buf.size() >= 400) {
      StsResult sts = dsp.detect_sts(ring_buf.data(), ring_buf.size());
      if (sts.trig_idx < 0) {
        if (ring_buf.size() > 48) {
          ring_buf.erase(ring_buf.begin(), ring_buf.end() - 48);
        }
        break;
      }

      const size_t burst_start = (sts.trig_idx >= 32) ? static_cast<size_t>(sts.trig_idx - 32) : 0;
      if (burst_start + 400 > ring_buf.size()) {
        break;
      }

      std::span<const c32> burst_span(&ring_buf[burst_start], ring_buf.size() - burst_start);
      FrameResult fr = dsp.decode_burst(burst_span, &sts);

      if (fr.signal_ok && fr.needed_samples > 0 &&
          burst_start + static_cast<size_t>(fr.needed_samples) > ring_buf.size()) {
        // Need to wait for full packet symbols to be read from stream
        break;
      }

      // Only emit genuine frames: either valid FCS or high-confidence preamble with reasonable CFO
      const bool genuine_burst = fr.fcs_ok || (!fcs_only && fr.ltf_peak >= 0.012f && fr.snr_db >= 2.0f &&
                                              std::abs(fr.coarse_cfo_hz + fr.fine_cfo_hz) < 150000.f);

      if (fr.signal_ok && !fr.payload.empty() && genuine_burst) {
        total_packets++;
        if (fr.fcs_ok) valid_fcs_count++;

        auto pkt_info = WifiMac::parse_frame(fr.payload, fr.fcs_ok);
        pkt_info.rate_name = fr.rate_name;
        pkt_info.rate_val = fr.rate_val;
        pkt_info.snr_db = fr.snr_db;
        pkt_info.evm_db = fr.evm_db;
        pkt_info.cfo_hz = fr.coarse_cfo_hz + fr.fine_cfo_hz;
        pkt_info.ltf_peak = fr.ltf_peak;

        const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        pkt_info.timestamp_us = now_us;

        if (stdout_json) {
          const bool send_const = include_constellation && !fr.eq_symbols.empty();
          std::cout << pkt_info.to_json(send_const, send_const ? fr.eq_symbols : std::vector<c32>{}) << "\n" << std::flush;
        } else if (pretty) {
          std::cout << "[" << std::setw(5) << total_packets << "] "
                    << (fr.fcs_ok ? "\033[1;32m[CRC OK]\033[0m " : "\033[1;31m[CRC BAD]\033[0m ")
                    << std::left << std::setw(15) << pkt_info.type_name << " "
                    << std::setw(18) << pkt_info.subtype_name << " "
                    << "Rate: " << std::setw(18) << fr.rate_name << " "
                    << "Len: " << std::setw(4) << fr.payload.size() << " B "
                    << "SNR: " << std::fixed << std::setprecision(1) << std::setw(4) << fr.snr_db << " dB "
                    << "EVM: " << std::setw(5) << fr.evm_db << " dB "
                    << "CFO: " << std::setw(6) << std::setprecision(0) << (fr.coarse_cfo_hz + fr.fine_cfo_hz) << " Hz\n";

          if (!pkt_info.summary.empty()) {
            std::cout << "      └─ \033[1;36m" << pkt_info.summary << "\033[0m\n";
          }
        }

        if (max_packets > 0 && total_packets >= static_cast<uint64_t>(max_packets)) {
          g_running = false;
          break;
        }
      }

      // Advance past this burst
      const size_t advance = (fr.signal_ok && fr.needed_samples > 0)
                                 ? std::min(ring_buf.size(), burst_start + static_cast<size_t>(fr.needed_samples))
                                 : std::min(ring_buf.size(), burst_start + 160);
      ring_buf.erase(ring_buf.begin(), ring_buf.begin() + advance);
    }
  }

  if (pretty) {
    std::cout << "---------------------------------------------------------\n"
              << "Summary: " << total_packets << " frames decoded, "
              << valid_fcs_count << " FCS valid ("
              << (total_packets ? (valid_fcs_count * 100 / total_packets) : 0) << "%)\n"
              << "=========================================================\n";
  }

  return 0;
}
