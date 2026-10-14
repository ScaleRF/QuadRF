#!/usr/bin/env python3
"""
QuadRF 802.11 ALFA Wi-Fi Adapter Compatibility Bridge
Bridges standard Linux Wi-Fi adapters in monitor mode (e.g. ALFA AWUS036ACH / wlp198s0f0u2)
to the QuadRF 802.11 OFDM No-ACK QoS Data protocol over the air.

Features:
- Encapsulates Linux TUN IP packets into IEEE 802.11 QoS Data frames (No-ACK policy 0x0088)
- Decapsulates QuadRF 802.11 QoS Data frames into Linux TUN IP packets
- Supports QuadRF Peer Chat protocol (EtherType 0x88B5)
- Pure Python implementation (requires no external dependencies like Scapy)
- Configures monitor mode and channel frequency
"""

import argparse
import fcntl
import json
import os
import select
import socket
import struct
import subprocess
import sys
import time
import zlib

# Linux TUN constants
IFF_TUN = 0x0001
IFF_NO_PI = 0x1000
TUNSETIFF = 0x400454CA

# IEEE 802.11 Constants
FC_QOS_DATA = 0x0088
QOS_CTRL_NO_ACK = 0x0020
LLC_SNAP_PREFIX = bytes([0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00])
ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_CHAT = 0x88B5
DEFAULT_BSSID = bytes([0x02, 0x00, 0x0A, 0x63, 0x00, 0x00])

# Radiotap Header for TX injection (Rate: 6 Mbps)
# it_version=0, it_pad=0, it_len=14
# it_present = IEEE80211_RADIOTAP_RATE (bit 2) | IEEE80211_RADIOTAP_TX_FLAGS (bit 15)
RADIOTAP_TX_6MBPS = bytes([
    0x00, 0x00,              # version 0, pad 0
    0x0E, 0x00,              # header length: 14 bytes
    0x04, 0x80, 0x00, 0x00,  # flags: bit 2 (Rate), bit 15 (TX Flags)
    0x0C,                    # Rate: 6.0 Mbps (12 in units of 500 kbps)
    0x00,                    # padding
    0x18, 0x00,              # TX flags: NO_ACK (0x0008) | NO_SEQ (0x0010) = 0x0018
    0x00, 0x00               # padding to 14 bytes
])

def wifi_crc32(data: bytes) -> int:
    """Standard IEEE 802.11 32-bit CRC (same as ethernet/gzip CRC32)."""
    return zlib.crc32(data) & 0xFFFFFFFF

def mac_to_bytes(mac_str: str) -> bytes:
    return bytes(int(b, 16) for b in mac_str.split(':'))

def bytes_to_mac(b: bytes) -> str:
    return ':'.join(f'{x:02x}' for x in b[:6])

class AlfaWifiBridge:
    def __init__(self, iface: str, freq_mhz: int, my_ip: str, my_mac: str,
                 peer_ip: str, peer_mac: str, tun_name: str = 'tun-alfa'):
        self.iface = iface
        self.freq_mhz = freq_mhz
        self.my_ip = my_ip
        self.my_mac = mac_to_bytes(my_mac)
        self.peer_ip = peer_ip
        self.peer_mac = mac_to_bytes(peer_mac)
        self.tun_name = tun_name
        self.seq_num = 0

        self.tun_fd = -1
        self.raw_sock = None
        self.running = False

    def setup_monitor_interface(self):
        print(f"[BRIDGE] Setting {self.iface} to monitor mode on {self.freq_mhz} MHz...")
        subprocess.run(["ip", "link", "set", self.iface, "down"], check=False)
        subprocess.run(["iw", "dev", self.iface, "set", "type", "monitor"], check=False)
        subprocess.run(["ip", "link", "set", self.iface, "up"], check=False)
        res = subprocess.run(["iw", "dev", self.iface, "set", "freq", str(self.freq_mhz)], capture_output=True, text=True)
        if res.returncode != 0:
            print(f"[BRIDGE] Warning: iw set freq failed: {res.stderr.strip()}")
            print(f"[BRIDGE] Attempting channel mapping for {self.freq_mhz} MHz...")
            chan = (self.freq_mhz - 5000) // 5
            subprocess.run(["iw", "dev", self.iface, "set", "channel", str(chan)], check=False)

    def open_tun(self):
        print(f"[BRIDGE] Opening TUN interface {self.tun_name} with IP {self.my_ip}/24...")
        self.tun_fd = os.open('/dev/net/tun', os.O_RDWR)
        ifr = struct.pack('16sH', self.tun_name.encode('ascii'), IFF_TUN | IFF_NO_PI)
        fcntl.ioctl(self.tun_fd, TUNSETIFF, ifr)

        # Configure IP and MTU
        subprocess.run(["ip", "addr", "add", f"{self.my_ip}/24", "dev", self.tun_name], check=False)
        subprocess.run(["ip", "link", "set", "dev", self.tun_name, "mtu", "1500", "up"], check=False)
        print(f"[BRIDGE] TUN interface {self.tun_name} is UP at {self.my_ip}")

    def open_raw_socket(self):
        print(f"[BRIDGE] Binding raw packet socket to {self.iface} (ETH_P_ALL)...")
        self.raw_sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
        self.raw_sock.bind((self.iface, 0))

    def build_80211_frame(self, payload: bytes, ethertype: int, da: bytes) -> bytes:
        """Build standard IEEE 802.11 QoS Data frame with No-ACK policy."""
        # 1. MAC Header (26 bytes for QoS Data)
        fc = struct.pack('<H', FC_QOS_DATA)
        duration = struct.pack('<H', 0x0000)
        addr1_da = da
        addr2_sa = self.my_mac
        addr3_bssid = DEFAULT_BSSID
        seq_ctrl = struct.pack('<H', (self.seq_num << 4) & 0xFFF0)
        self.seq_num = (self.seq_num + 1) & 0x0FFF
        qos_ctrl = struct.pack('<H', QOS_CTRL_NO_ACK)

        mac_hdr = fc + duration + addr1_da + addr2_sa + addr3_bssid + seq_ctrl + qos_ctrl

        # 2. LLC/SNAP Header (8 bytes)
        llc_snap = LLC_SNAP_PREFIX + struct.pack('>H', ethertype)

        # 3. Assemble body and compute FCS
        body = mac_hdr + llc_snap + payload
        fcs = struct.pack('<I', wifi_crc32(body))

        # 4. Prepend Radiotap header for injection
        return RADIOTAP_TX_6MBPS + body + fcs

    def parse_80211_frame(self, raw_pkt: bytes):
        """Parse incoming Radiotap + 802.11 frame, returns (ethertype, payload, sa, da)."""
        if len(raw_pkt) < 4:
            return None

        # Parse Radiotap header length
        rt_len = struct.unpack('<H', raw_pkt[2:4])[0]
        if len(raw_pkt) < rt_len + 30:  # Need at least MAC hdr + LLC + FCS
            return None

        dot11 = raw_pkt[rt_len:]
        if len(dot11) < 30:
            return None

        # Frame Control
        fc = struct.unpack('<H', dot11[:2])[0]
        if (fc & 0x000C) != 0x0008:  # Type must be Data
            return None

        subtype = (fc >> 4) & 0x0F
        if subtype == 8:
            mac_hdr_len = 26  # QoS Data
        elif subtype == 0:
            mac_hdr_len = 24  # Standard Data
        else:
            return None

        if len(dot11) < mac_hdr_len + 8 + 4:
            return None

        da = dot11[4:10]
        sa = dot11[10:16]

        # Ignore self transmissions
        if sa == self.my_mac:
            return None

        # Verify FCS if present
        body_len = len(dot11) - 4
        calc_crc = wifi_crc32(dot11[:body_len])
        rx_crc = struct.unpack('<I', dot11[body_len:body_len + 4])[0]
        # Some drivers strip FCS; if non-zero check match
        if rx_crc != 0 and calc_crc != rx_crc:
            return None

        # Verify LLC / SNAP
        llc = dot11[mac_hdr_len:mac_hdr_len + 8]
        if llc[:6] != LLC_SNAP_PREFIX:
            return None

        ethertype = struct.unpack('>H', llc[6:8])[0]
        payload = dot11[mac_hdr_len + 8:body_len]
        return ethertype, payload, sa, da

    def send_chat(self, text: str):
        """Transmit QuadRF peer chat message."""
        sender = b"alfa-host"
        slen = len(sender)
        t_bytes = text.encode('utf-8')[:1024]
        tlen = len(t_bytes)
        payload = b'QCHT' + bytes([slen]) + sender + struct.pack('>H', tlen) + t_bytes
        frame = self.build_80211_frame(payload, ETHERTYPE_CHAT, self.peer_mac)
        self.raw_sock.send(frame)
        print(f"[BRIDGE TX CHAT] {text}")

    def run(self):
        self.setup_monitor_interface()
        self.open_tun()
        self.open_raw_socket()
        self.running = True

        print(f"[BRIDGE] Running full-duplex tunnel {self.my_ip} <-> {self.peer_ip} on {self.iface} ({self.freq_mhz} MHz)")
        print("[BRIDGE] Press Ctrl+C to stop.")

        try:
            while self.running:
                rlist, _, _ = select.select([self.tun_fd, self.raw_sock], [], [], 1.0)
                for fd in rlist:
                    if fd == self.tun_fd:
                        # Packet from local Linux TUN -> transmit over air
                        ip_pkt = os.read(self.tun_fd, 2048)
                        if ip_pkt and len(ip_pkt) >= 20:
                            frame = self.build_80211_frame(ip_pkt, ETHERTYPE_IPV4, self.peer_mac)
                            self.raw_sock.send(frame)

                    elif fd == self.raw_sock:
                        # Frame received over air -> inject into TUN or handle chat
                        raw_pkt = self.raw_sock.recv(4096)
                        parsed = self.parse_80211_frame(raw_pkt)
                        if parsed:
                            ethertype, payload, sa, da = parsed
                            if ethertype == ETHERTYPE_IPV4:
                                os.write(self.tun_fd, payload)
                            elif ethertype == ETHERTYPE_CHAT:
                                if payload.startswith(b'QCHT') and len(payload) >= 7:
                                    slen = payload[4]
                                    sender = payload[5:5 + slen].decode('utf-8', errors='ignore')
                                    tlen = struct.unpack('>H', payload[5 + slen:7 + slen])[0]
                                    msg = payload[7 + slen:7 + slen + tlen].decode('utf-8', errors='ignore')
                                    print(f"\n[PEER CHAT from {sender}]: {msg}\n> ", end='', flush=True)

        except KeyboardInterrupt:
            print("\n[BRIDGE] Stopping...")
        finally:
            if self.tun_fd >= 0:
                os.close(self.tun_fd)
            if self.raw_sock:
                self.raw_sock.close()

def main():
    parser = argparse.ArgumentParser(description="QuadRF 802.11 ALFA Wi-Fi Bridge")
    parser.add_argument("-i", "--iface", default="wlp198s0f0u2", help="Wireless interface")
    parser.add_argument("-f", "--freq", type=int, default=5860, help="Carrier frequency in MHz (default: 5860)")
    parser.add_argument("--my-ip", default="10.99.0.1", help="Local TUN IP (default: 10.99.0.1)")
    parser.add_argument("--my-mac", default="00:c0:ca:a9:7f:ae", help="Local Wi-Fi MAC")
    parser.add_argument("--peer-ip", default="10.99.0.2", help="Peer IP (default: 10.99.0.2)")
    parser.add_argument("--peer-mac", default="02:00:0a:63:00:02", help="Peer MAC (default: 02:00:0a:63:00:02)")
    parser.add_argument("--tun", default="tun-alfa", help="TUN device name")
    args = parser.parse_args()

    bridge = AlfaWifiBridge(args.iface, args.freq, args.my_ip, args.my_mac,
                            args.peer_ip, args.peer_mac, args.tun)
    bridge.run()

if __name__ == '__main__':
    main()
