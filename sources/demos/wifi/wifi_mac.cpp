#include "wifi_mac.hpp"
#include "wifi_viterbi.hpp"

#include <complex>
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
  char buf[20];
  std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  return std::string(buf);
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

std::string WifiMac::format_ip(const uint8_t* ip_bytes) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
  return std::string(buf);
}

namespace {

std::string json_escape(const std::string& s) {
  std::ostringstream o;
  for (char c : s) {
    if (c == '"') o << "\\\"";
    else if (c == '\\') o << "\\\\";
    else if (c == '\b') o << "\\b";
    else if (c == '\f') o << "\\f";
    else if (c == '\n') o << "\\n";
    else if (c == '\r') o << "\\r";
    else if (c == '\t') o << "\\t";
    else if (static_cast<unsigned char>(c) < 32 || static_cast<unsigned char>(c) >= 127) {
      char hx[8];
      std::snprintf(hx, sizeof(hx), "\\u%04x", static_cast<unsigned char>(c));
      o << hx;
    } else {
      o << c;
    }
  }
  return o.str();
}

}  // namespace

std::string WifiPacketInfo::to_json(bool include_constellation,
                                    const std::vector<std::complex<float>>& constell) const {
  std::ostringstream j;
  j << "{\"type\":\"wifi_pkt\""
    << ",\"fcs_ok\":" << (fcs_ok ? "true" : "false")
    << ",\"pkt_type\":\"" << type_name << "\""
    << ",\"subtype\":\"" << subtype_name << "\""
    << ",\"type_val\":" << static_cast<int>(type)
    << ",\"subtype_val\":" << static_cast<int>(subtype)
    << ",\"fc\":\"0x" << std::hex << std::setfill('0') << std::setw(4) << fc << std::dec << "\"";

  if (has_da) j << ",\"da\":\"" << da_str << "\"";
  if (has_sa) j << ",\"sa\":\"" << sa_str << "\"";
  if (has_bssid) j << ",\"bssid\":\"" << bssid_str << "\"";
  if (!ssid.empty()) j << ",\"ssid\":\"" << json_escape(ssid) << "\"";

  j << ",\"seq\":" << seq_num
    << ",\"frag\":" << static_cast<int>(frag_num)
    << ",\"to_ds\":" << (to_ds ? "true" : "false")
    << ",\"from_ds\":" << (from_ds ? "true" : "false")
    << ",\"retry\":" << (retry ? "true" : "false")
    << ",\"protected\":" << (is_protected ? "true" : "false");

  if (channel > 0) j << ",\"channel\":" << channel;
  if (beacon_interval_tu > 0) j << ",\"beacon_int_tu\":" << beacon_interval_tu;

  if (has_qos) j << ",\"qos_tid\":" << static_cast<int>(tid);

  if (has_llc) {
    j << ",\"has_llc\":true";
    if (ethertype > 0) {
      j << ",\"ethertype\":\"0x" << std::hex << std::setfill('0') << std::setw(4) << ethertype << std::dec << "\""
        << ",\"ethertype_name\":\"" << ethertype_name << "\"";
    }
  }

  if (is_ipv4) {
    j << ",\"is_ipv4\":true"
      << ",\"ip_src\":\"" << ip_src << "\""
      << ",\"ip_dst\":\"" << ip_dst << "\""
      << ",\"ip_proto\":\"" << ip_proto_name << "\""
      << ",\"ip_len\":" << ip_len;
    if (port_src > 0 || port_dst > 0) {
      j << ",\"port_src\":" << port_src
        << ",\"port_dst\":" << port_dst;
    }
  }

  // PHY Metrics
  j << ",\"rate\":\"" << rate_name << "\""
    << ",\"rate_val\":" << rate_val
    << ",\"len\":" << length_bytes
    << ",\"snr_db\":" << std::fixed << std::setprecision(1) << snr_db
    << ",\"evm_db\":" << std::fixed << std::setprecision(1) << evm_db
    << ",\"cfo_hz\":" << std::fixed << std::setprecision(0) << cfo_hz
    << ",\"ltf_peak\":" << std::fixed << std::setprecision(3) << ltf_peak;

  if (!summary.empty()) {
    j << ",\"summary\":\"" << json_escape(summary) << "\"";
  }

  if (include_constellation && !constell.empty()) {
    j << ",\"constellation\":[";
    // Subsample constellation points to at most 128 points for smooth web rendering
    const size_t step = std::max<size_t>(1, constell.size() / 128);
    bool first = true;
    for (size_t i = 0; i < constell.size(); i += step) {
      if (!first) j << ",";
      first = false;
      j << "[" << std::setprecision(3) << constell[i].real() << ","
        << std::setprecision(3) << constell[i].imag() << "]";
    }
    j << "]";
  }

  j << "}";
  return j.str();
}

WifiPacketInfo WifiMac::parse_frame(std::span<const uint8_t> mpdu, bool fcs_verified) {
  WifiPacketInfo info;
  info.length_bytes = static_cast<int>(mpdu.size());
  if (mpdu.size() < 10) return info;

  // 1. FCS Verification (last 4 bytes)
  if (mpdu.size() >= 14) {
    const size_t body_len = mpdu.size() - 4;
    info.calc_crc = wifi_crc32(mpdu.data(), body_len);
    std::memcpy(&info.rx_crc, &mpdu[body_len], 4);
    info.fcs_ok = fcs_verified || (info.calc_crc == info.rx_crc);
  }

  // 2. Frame Control
  info.fc = static_cast<uint16_t>(mpdu[0]) | (static_cast<uint16_t>(mpdu[1]) << 8);
  info.protocol_version = mpdu[0] & 0x03;
  info.type = (mpdu[0] >> 2) & 0x03;
  info.subtype = (mpdu[0] >> 4) & 0x0F;

  // Flags from byte 1
  info.to_ds = (mpdu[1] & 0x01) != 0;
  info.from_ds = (mpdu[1] & 0x02) != 0;
  info.more_frag = (mpdu[1] & 0x04) != 0;
  info.retry = (mpdu[1] & 0x08) != 0;
  info.pwr_mgmt = (mpdu[1] & 0x10) != 0;
  info.more_data = (mpdu[1] & 0x20) != 0;
  info.is_protected = (mpdu[1] & 0x40) != 0;
  info.order = (mpdu[1] & 0x80) != 0;

  info.duration_id = static_cast<uint16_t>(mpdu[2]) | (static_cast<uint16_t>(mpdu[3]) << 8);

  // Type & Subtype Names
  switch (info.type) {
    case 0: {  // Management
      info.type_name = "Management";
      switch (info.subtype) {
        case 0:  info.subtype_name = "Assoc Request"; break;
        case 1:  info.subtype_name = "Assoc Response"; break;
        case 2:  info.subtype_name = "Reassoc Request"; break;
        case 3:  info.subtype_name = "Reassoc Response"; break;
        case 4:  info.subtype_name = "Probe Request"; break;
        case 5:  info.subtype_name = "Probe Response"; break;
        case 8:  info.subtype_name = "Beacon"; break;
        case 9:  info.subtype_name = "ATIM"; break;
        case 10: info.subtype_name = "Disassociation"; break;
        case 11: info.subtype_name = "Authentication"; break;
        case 12: info.subtype_name = "Deauthentication"; break;
        case 13: info.subtype_name = "Action"; break;
        case 14: info.subtype_name = "Action No ACK"; break;
        default: info.subtype_name = "Subtype " + std::to_string(info.subtype); break;
      }
      break;
    }
    case 1: {  // Control
      info.type_name = "Control";
      switch (info.subtype) {
        case 4:  info.subtype_name = "Beamforming Report"; break;
        case 5:  info.subtype_name = "VHT NDP Announce"; break;
        case 7:  info.subtype_name = "Control Wrapper"; break;
        case 8:  info.subtype_name = "Block ACK Req"; break;
        case 9:  info.subtype_name = "Block ACK"; break;
        case 10: info.subtype_name = "PS-Poll"; break;
        case 11: info.subtype_name = "RTS"; break;
        case 12: info.subtype_name = "CTS"; break;
        case 13: info.subtype_name = "ACK"; break;
        case 14: info.subtype_name = "CF-End"; break;
        case 15: info.subtype_name = "CF-End + CF-Ack"; break;
        default: info.subtype_name = "Subtype " + std::to_string(info.subtype); break;
      }
      break;
    }
    case 2: {  // Data
      info.type_name = "Data";
      switch (info.subtype) {
        case 0:  info.subtype_name = "Data"; break;
        case 1:  info.subtype_name = "Data + CF-Ack"; break;
        case 2:  info.subtype_name = "Data + CF-Poll"; break;
        case 3:  info.subtype_name = "Data + CF-Ack + CF-Poll"; break;
        case 4:  info.subtype_name = "Null Data"; break;
        case 5:  info.subtype_name = "CF-Ack"; break;
        case 6:  info.subtype_name = "CF-Poll"; break;
        case 7:  info.subtype_name = "CF-Ack + CF-Poll"; break;
        case 8:  info.subtype_name = "QoS Data"; break;
        case 9:  info.subtype_name = "QoS Data + CF-Ack"; break;
        case 10: info.subtype_name = "QoS Data + CF-Poll"; break;
        case 11: info.subtype_name = "QoS Data + CF-Ack + CF-Poll"; break;
        case 12: info.subtype_name = "QoS Null"; break;
        case 14: info.subtype_name = "QoS CF-Poll"; break;
        default: info.subtype_name = "Subtype " + std::to_string(info.subtype); break;
      }
      break;
    }
    case 3: {  // Extension
      info.type_name = "Extension";
      info.subtype_name = "Subtype " + std::to_string(info.subtype);
      break;
    }
  }

  // 3. Address Resolution
  if (info.type == 0) {  // Management: Addr1=DA, Addr2=SA, Addr3=BSSID
    if (mpdu.size() >= 24) {
      info.has_da = true;   std::memcpy(info.da.data(), &mpdu[4], 6);
      info.has_sa = true;   std::memcpy(info.sa.data(), &mpdu[10], 6);
      info.has_bssid = true;std::memcpy(info.bssid.data(), &mpdu[16], 6);
      info.da_str = format_mac(info.da);
      info.sa_str = format_mac(info.sa);
      info.bssid_str = format_mac(info.bssid);

      const uint16_t sc = static_cast<uint16_t>(mpdu[22]) | (static_cast<uint16_t>(mpdu[23]) << 8);
      info.frag_num = sc & 0x0F;
      info.seq_num = sc >> 4;
    }
  } else if (info.type == 1) {  // Control
    if (info.subtype == 11 || info.subtype == 10 || info.subtype == 8 || info.subtype == 9) {
      if (mpdu.size() >= 16) {
        info.has_da = true; std::memcpy(info.da.data(), &mpdu[4], 6);
        info.has_sa = true; std::memcpy(info.sa.data(), &mpdu[10], 6);
        info.da_str = format_mac(info.da);
        info.sa_str = format_mac(info.sa);
      }
    } else if (info.subtype == 12 || info.subtype == 13) {  // CTS, ACK
      if (mpdu.size() >= 10) {
        info.has_da = true; std::memcpy(info.da.data(), &mpdu[4], 6);
        info.da_str = format_mac(info.da);
      }
    }
  } else if (info.type == 2) {  // Data
    if (mpdu.size() >= 24) {
      MacAddr a1, a2, a3;
      std::memcpy(a1.data(), &mpdu[4], 6);
      std::memcpy(a2.data(), &mpdu[10], 6);
      std::memcpy(a3.data(), &mpdu[16], 6);

      if (!info.to_ds && !info.from_ds) {
        info.has_da = true;    info.da = a1;
        info.has_sa = true;    info.sa = a2;
        info.has_bssid = true; info.bssid = a3;
      } else if (info.to_ds && !info.from_ds) {
        info.has_bssid = true; info.bssid = a1;
        info.has_sa = true;    info.sa = a2;
        info.has_da = true;    info.da = a3;
      } else if (!info.to_ds && info.from_ds) {
        info.has_da = true;    info.da = a1;
        info.has_bssid = true; info.bssid = a2;
        info.has_sa = true;    info.sa = a3;
      } else {
        info.has_da = true;    info.da = a3;
        info.has_sa = true;    info.sa = a2;
      }
      info.da_str = format_mac(info.da);
      info.sa_str = format_mac(info.sa);
      info.bssid_str = format_mac(info.bssid);

      const uint16_t sc = static_cast<uint16_t>(mpdu[22]) | (static_cast<uint16_t>(mpdu[23]) << 8);
      info.frag_num = sc & 0x0F;
      info.seq_num = sc >> 4;
    }
  }

  // 4. Management Parameter Tag Parsing (SSID, etc.)
  if (info.type == 0) {
    size_t tag_offset = 0;
    if ((info.subtype == 8 || info.subtype == 5) && mpdu.size() >= 38) {
      // Beacon (8) or Probe Response (5): 24 byte MAC hdr + 8B timestamp + 2B beacon int + 2B caps
      info.beacon_interval_tu = static_cast<uint16_t>(mpdu[32]) | (static_cast<uint16_t>(mpdu[33]) << 8);
      info.capabilities = static_cast<uint16_t>(mpdu[34]) | (static_cast<uint16_t>(mpdu[35]) << 8);
      tag_offset = 36;
    } else if (info.subtype == 4 && mpdu.size() >= 26) {
      // Probe Request: tags start at byte 24
      tag_offset = 24;
    }

    const size_t end_offset = mpdu.size() >= 4 ? mpdu.size() - 4 : mpdu.size();
    while (tag_offset + 2 <= end_offset) {
      const uint8_t tag_id = mpdu[tag_offset];
      const uint8_t tag_len = mpdu[tag_offset + 1];
      if (tag_offset + 2 + tag_len > end_offset) break;

      const uint8_t* val = &mpdu[tag_offset + 2];
      if (tag_id == 0 && tag_len <= 32) {  // SSID (max 32 octets per IEEE 802.11)
        bool printable = true;
        for (int k = 0; k < tag_len; ++k) {
          const uint8_t c = val[k];
          if (c < 32 || c > 126) {
            printable = false;
            break;
          }
        }
        if (printable) {
          // If FCS didn't pass, only accept SSIDs from Beacon or Probe frames
          if (info.fcs_ok || info.subtype == 8 || info.subtype == 5 || info.subtype == 4) {
            info.ssid = std::string(reinterpret_cast<const char*>(val), tag_len);
          }
        }
      } else if (tag_id == 1 || tag_id == 50) {  // Supported Rates
        for (int r = 0; r < tag_len; ++r) {
          const int rate_500k = val[r] & 0x7F;
          info.supported_rates_mbps.push_back(rate_500k / 2);
        }
      } else if (tag_id == 3 && tag_len >= 1) {  // DS Parameter Set (Current Channel)
        info.channel = static_cast<int>(val[0]);
      }

      tag_offset += 2 + tag_len;
    }
  }

  // 5. Data Frame Payload & LLC/SNAP/IP Parsing
  if (info.type == 2) {
    size_t mac_hdr_len = 24;
    if (info.subtype == 8 || info.subtype == 9 || info.subtype == 10 || info.subtype == 11) {
      // QoS Data frames include 2-byte QoS Control
      if (mpdu.size() >= 26) {
        info.has_qos = true;
        info.qos_ctrl = static_cast<uint16_t>(mpdu[24]) | (static_cast<uint16_t>(mpdu[25]) << 8);
        info.tid = info.qos_ctrl & 0x0F;
        mac_hdr_len = 26;
      }
    }
    if (info.to_ds && info.from_ds) {
      mac_hdr_len += 6;  // 4th address present
    }

    const size_t end_offset = mpdu.size() >= 4 ? mpdu.size() - 4 : mpdu.size();
    if (mac_hdr_len + 8 <= end_offset) {
      // Check for 802.2 LLC/SNAP header (AA AA 03 00 00 00)
      if (mpdu[mac_hdr_len] == 0xAA && mpdu[mac_hdr_len + 1] == 0xAA && mpdu[mac_hdr_len + 2] == 0x03) {
        info.has_llc = true;
        info.ethertype = (static_cast<uint16_t>(mpdu[mac_hdr_len + 6]) << 8) |
                          static_cast<uint16_t>(mpdu[mac_hdr_len + 7]);

        if (info.ethertype == 0x0800) info.ethertype_name = "IPv4";
        else if (info.ethertype == 0x86DD) info.ethertype_name = "IPv6";
        else if (info.ethertype == 0x0806) info.ethertype_name = "ARP";
        else if (info.ethertype == 0x888E) info.ethertype_name = "EAPOL";
        else info.ethertype_name = "EtherType 0x" + std::to_string(info.ethertype);

        // Parse IPv4
        const size_t ip_offset = mac_hdr_len + 8;
        if (info.ethertype == 0x0800 && ip_offset + 20 <= end_offset) {
          const uint8_t ver_ihl = mpdu[ip_offset];
          if ((ver_ihl >> 4) == 4) {
            info.is_ipv4 = true;
            const size_t ihl = (ver_ihl & 0x0F) * 4;
            info.ip_len = (static_cast<uint16_t>(mpdu[ip_offset + 2]) << 8) |
                           static_cast<uint16_t>(mpdu[ip_offset + 3]);
            info.ip_proto = mpdu[ip_offset + 9];
            info.ip_src = format_ip(&mpdu[ip_offset + 12]);
            info.ip_dst = format_ip(&mpdu[ip_offset + 16]);

            if (info.ip_proto == 1) info.ip_proto_name = "ICMP";
            else if (info.ip_proto == 6) info.ip_proto_name = "TCP";
            else if (info.ip_proto == 17) info.ip_proto_name = "UDP";
            else info.ip_proto_name = "Proto " + std::to_string(info.ip_proto);

            // TCP / UDP Ports
            const size_t transport_offset = ip_offset + ihl;
            if ((info.ip_proto == 6 || info.ip_proto == 17) && transport_offset + 4 <= end_offset) {
              info.port_src = (static_cast<uint16_t>(mpdu[transport_offset + 0]) << 8) |
                               static_cast<uint16_t>(mpdu[transport_offset + 1]);
              info.port_dst = (static_cast<uint16_t>(mpdu[transport_offset + 2]) << 8) |
                               static_cast<uint16_t>(mpdu[transport_offset + 3]);
            }
          }
        }
      }
    }
  }

  // 6. Construct Human-Readable Summary
  std::ostringstream s;
  if (!info.ssid.empty()) {
    s << info.subtype_name << " SSID:\"" << info.ssid << "\"";
    if (info.channel > 0) s << " [Ch " << info.channel << "]";
    if (info.has_bssid) s << " BSSID:" << info.bssid_str;
  } else if (info.is_ipv4) {
    s << info.subtype_name << " " << info.ip_proto_name << " "
      << info.ip_src;
    if (info.port_src > 0) s << ":" << info.port_src;
    s << " -> " << info.ip_dst;
    if (info.port_dst > 0) s << ":" << info.port_dst;
  } else if (info.has_llc && !info.ethertype_name.empty()) {
    s << info.subtype_name << " (" << info.ethertype_name << ") " << info.sa_str << " -> " << info.da_str;
  } else if (info.has_sa && info.has_da) {
    s << info.subtype_name << " " << info.sa_str << " -> " << info.da_str;
  } else if (info.has_da) {
    s << info.subtype_name << " -> " << info.da_str;
  } else {
    s << info.subtype_name << " (" << info.length_bytes << " B)";
  }
  info.summary = s.str();

  return info;
}

}  // namespace quadrf_wifi

