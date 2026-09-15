#pragma once

#include <array>
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

class WifiMac {
 public:
  static std::vector<uint8_t> encapsulate_ipv4(std::span<const uint8_t> ip_packet,
                                               uint16_t seq_num,
                                               const MacTxConfig& cfg = {});
  static std::vector<uint8_t> decapsulate_ipv4(std::span<const uint8_t> mpdu,
                                               const MacRxConfig& cfg = {});

  static std::string format_mac(const MacAddr& addr);
  static MacAddr parse_mac(const std::string& str);
};

}  // namespace quadrf_wifi
