#pragma once

#include <array>
#include <complex>
#include <cstdint>
#include <span>
#include <string>
#include <vector>


namespace quadrf_wifi {

using MacAddr = std::array<uint8_t, 6>;

enum class FrameFormat {
  IbssData,   // Standard 802.11 Data (FC 0x0008, 24-byte MAC header)
  QosData     // 802.11 QoS Data with No-ACK policy (FC 0x0088, 26-byte MAC header)
};

struct MacTxConfig {
  FrameFormat format = FrameFormat::QosData;
  MacAddr da = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast by default
  MacAddr sa = {0x00, 0x12, 0x34, 0x56, 0x78, 0x9A};  // QuadRF node local MAC
  MacAddr bssid = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
  uint16_t ethertype = 0x0800;  // IPv4
};

struct MacRxConfig {
  MacAddr my_mac = {0x00, 0x12, 0x34, 0x56, 0x78, 0x9A};
  bool accept_all_da = true;  // Accept broadcast and all peer frames
  bool drop_self_packets = false;  // Drop frames originating from my_mac (self-TX loopback)
  uint16_t accept_ethertype = 0x0800;
};

struct WifiPacketInfo {
  bool fcs_ok = false;
  uint32_t calc_crc = 0;
  uint32_t rx_crc = 0;

  // Frame Control
  uint16_t fc = 0;
  uint8_t protocol_version = 0;
  uint8_t type = 0;          // 0 = Mgmt, 1 = Ctrl, 2 = Data, 3 = Ext
  uint8_t subtype = 0;       // 0..15
  std::string type_name;     // "Management", "Control", "Data"
  std::string subtype_name;  // "Beacon", "Probe Request", "QoS Data", etc.

  // Flags
  bool to_ds = false;
  bool from_ds = false;
  bool more_frag = false;
  bool retry = false;
  bool pwr_mgmt = false;
  bool more_data = false;
  bool is_protected = false;
  bool order = false;

  uint16_t duration_id = 0;

  // Addresses
  bool has_da = false;
  bool has_sa = false;
  bool has_bssid = false;
  MacAddr da{0};
  MacAddr sa{0};
  MacAddr bssid{0};
  std::string da_str;
  std::string sa_str;
  std::string bssid_str;

  // Sequence Control
  uint16_t seq_num = 0;
  uint8_t frag_num = 0;

  // QoS Control
  bool has_qos = false;
  uint16_t qos_ctrl = 0;
  uint8_t tid = 0;

  // Management Fields
  std::string ssid;
  int channel = 0;
  int beacon_interval_tu = 0;
  uint16_t capabilities = 0;
  std::vector<int> supported_rates_mbps;

  // LLC / SNAP / Network Layer
  bool has_llc = false;
  uint16_t ethertype = 0;
  std::string ethertype_name;

  // IPv4 Header
  bool is_ipv4 = false;
  std::string ip_src;
  std::string ip_dst;
  uint8_t ip_proto = 0;
  std::string ip_proto_name;
  uint16_t ip_len = 0;
  uint16_t port_src = 0;
  uint16_t port_dst = 0;

  // PHY / DSP Metrics
  std::string rate_name;
  int rate_val = 0;
  int length_bytes = 0;
  float snr_db = 0.0f;
  float evm_db = 0.0f;
  float cfo_hz = 0.0f;
  float ltf_peak = 0.0f;
  int64_t timestamp_us = 0;

  // Formatted human-readable summary
  std::string summary;

  // JSON serialization for PhaseGaze UI and IPC
  std::string to_json(bool include_constellation = false,
                      const std::vector<std::complex<float>>& constell = {}) const;
};

class WifiMac {
 public:
  static std::vector<uint8_t> encapsulate_ipv4(std::span<const uint8_t> ip_packet,
                                               uint16_t seq_num,
                                               const MacTxConfig& cfg = {});
  static std::vector<uint8_t> decapsulate_ipv4(std::span<const uint8_t> mpdu,
                                               const MacRxConfig& cfg = {});

  static WifiPacketInfo parse_frame(std::span<const uint8_t> mpdu, bool fcs_verified = false);

  static std::string format_mac(const MacAddr& addr);
  static MacAddr parse_mac(const std::string& str);
  static std::string format_ip(const uint8_t* ip_bytes);
};

}  // namespace quadrf_wifi

