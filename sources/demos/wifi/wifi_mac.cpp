#include "wifi_mac.hpp"
#include "wifi_viterbi.hpp"

#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace quadrf_wifi {

std::vector<uint8_t> WifiMac::encapsulate_ipv4(std::span<const uint8_t> ip_packet,
                                               uint16_t seq_num,
                                               const MacTxConfig& cfg) {
  const bool is_qos = (cfg.format == FrameFormat::QosData);
  const size_t mac_hdr_len = is_qos ? 26 : 24;
  constexpr size_t llc_snap_len = 8;
  constexpr size_t fcs_len = 4;

  const size_t total_len = mac_hdr_len + llc_snap_len + ip_packet.size() + fcs_len;
  std::vector<uint8_t> mpdu(total_len);

  // 1. MAC Header
  // Frame Control: Subtype 8 (QoS Data) -> 0x0088, Subtype 0 (Data) -> 0x0008
  mpdu[0] = is_qos ? 0x88 : 0x08;
  mpdu[1] = 0x00;

  // Duration / ID
  mpdu[2] = 0x00;
  mpdu[3] = 0x00;

  // Addr1 (DA)
  std::memcpy(&mpdu[4], cfg.da.data(), 6);

  // Addr2 (SA)
  std::memcpy(&mpdu[10], cfg.sa.data(), 6);

  // Addr3 (BSSID)
  std::memcpy(&mpdu[16], cfg.bssid.data(), 6);

  // Sequence Control: (seq_num & 0xFFF) << 4
  const uint16_t seq_ctrl = (seq_num & 0x0FFF) << 4;
  mpdu[22] = static_cast<uint8_t>(seq_ctrl & 0xFF);
  mpdu[23] = static_cast<uint8_t>((seq_ctrl >> 8) & 0xFF);

  if (is_qos) {
    // QoS Control: 2 bytes
    // Bits 5-6: 01 (No-ACK policy = 0x0020)
    mpdu[24] = 0x20;
    mpdu[25] = 0x00;
  }

  // 2. LLC / SNAP Header (8 bytes)
  const size_t llc_offset = mac_hdr_len;
  mpdu[llc_offset + 0] = 0xAA;  // DSAP
  mpdu[llc_offset + 1] = 0xAA;  // SSAP
  mpdu[llc_offset + 2] = 0x03;  // Control (UI)
  mpdu[llc_offset + 3] = 0x00;  // OUI[0]
  mpdu[llc_offset + 4] = 0x00;  // OUI[1]
  mpdu[llc_offset + 5] = 0x00;  // OUI[2]
  mpdu[llc_offset + 6] = static_cast<uint8_t>((cfg.ethertype >> 8) & 0xFF);
  mpdu[llc_offset + 7] = static_cast<uint8_t>(cfg.ethertype & 0xFF);

  // 3. IP Payload
  const size_t payload_offset = mac_hdr_len + llc_snap_len;
  std::memcpy(&mpdu[payload_offset], ip_packet.data(), ip_packet.size());

  // 4. FCS CRC32 (over MAC header + LLC + Payload)
  const size_t body_len = mac_hdr_len + llc_snap_len + ip_packet.size();
  const uint32_t crc = wifi_crc32(mpdu.data(), body_len);
  std::memcpy(&mpdu[body_len], &crc, sizeof(crc));

  return mpdu;
}

std::vector<uint8_t> WifiMac::decapsulate_ipv4(std::span<const uint8_t> mpdu,
                                               const MacRxConfig& cfg) {
  if (mpdu.size() < 36) return {};

  // 1. Verify FCS CRC-32
  const size_t body_len = mpdu.size() - 4;
  const uint32_t calc_crc = wifi_crc32(mpdu.data(), body_len);
  uint32_t rx_crc = 0;
  std::memcpy(&rx_crc, &mpdu[body_len], sizeof(rx_crc));
  if (calc_crc != rx_crc) return {};

  // 2. Validate Frame Control
  const uint8_t fc0 = mpdu[0];
  if ((fc0 & 0x0C) != 0x08) return {};  // Type must be Data (0b10 = 2)

  const uint8_t subtype = (fc0 >> 4) & 0x0F;
  size_t mac_hdr_len = 24;
  if (subtype == 8) {  // QoS Data
    mac_hdr_len = 26;
  } else if (subtype == 0) {  // Standard Data
    mac_hdr_len = 24;
  } else {
    // Other data subtype (e.g. Null, CF-Ack, etc.)
    return {};
  }

  if (mpdu.size() < mac_hdr_len + 8 + 4) return {};

  // 3. Filter Source Address (Addr2: bytes 10..15)
  // Drop frames originating from my_mac (self-TX loopback in full-duplex)
  if (cfg.drop_self_packets) {
    bool is_from_me = true;
    for (int i = 0; i < 6; ++i) {
      if (mpdu[10 + i] != cfg.my_mac[static_cast<size_t>(i)]) {
        is_from_me = false;
        break;
      }
    }
    if (is_from_me) return {};
  }

  // 4. Filter Destination Address (Addr1: bytes 4..9)
  if (!cfg.accept_all_da) {
    bool is_bcast = true;
    bool is_mine = true;
    for (int i = 0; i < 6; ++i) {
      if (mpdu[4 + i] != 0xFF) is_bcast = false;
      if (mpdu[4 + i] != cfg.my_mac[static_cast<size_t>(i)]) is_mine = false;
    }
    if (!is_bcast && !is_mine) return {};
  }

  // 4. Validate LLC / SNAP Header
  const size_t llc_offset = mac_hdr_len;
  if (mpdu[llc_offset + 0] != 0xAA ||
      mpdu[llc_offset + 1] != 0xAA ||
      mpdu[llc_offset + 2] != 0x03) {
    // Not standard LLC/SNAP encapsulated IP
    return {};
  }

  const uint16_t ethertype = (static_cast<uint16_t>(mpdu[llc_offset + 6]) << 8) |
                             mpdu[llc_offset + 7];
  if (ethertype != cfg.accept_ethertype) return {};

  // 5. Extract IP Payload
  const size_t ip_offset = mac_hdr_len + 8;
  const size_t ip_len = body_len - ip_offset;
  if (ip_len == 0) return {};

  return std::vector<uint8_t>(mpdu.begin() + ip_offset, mpdu.begin() + body_len);
}

std::string WifiMac::format_mac(const MacAddr& addr) {
  std::ostringstream oss;
  for (size_t i = 0; i < addr.size(); ++i) {
    if (i > 0) oss << ":";
    oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(addr[i]);
  }
  return oss.str();
}

MacAddr WifiMac::parse_mac(const std::string& str) {
  MacAddr addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  unsigned int b[6];
  if (std::sscanf(str.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
                  &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
    for (int i = 0; i < 6; ++i) addr[static_cast<size_t>(i)] = static_cast<uint8_t>(b[i]);
  }
  return addr;
}

}  // namespace quadrf_wifi
