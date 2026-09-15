#include "wifi_radio.hpp"

#include <fcntl.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

namespace quadrf_wifi {

namespace {

#ifndef CSI_IOC_MAGIC
#define CSI_IOC_MAGIC 'C'
#endif

struct csi_jtag_reg {
  uint8_t addr;
  uint16_t value;
  uint16_t _pad;
};

#define CSI_IOC_JTAG_SETUP     _IO(CSI_IOC_MAGIC, 0x10)
#define CSI_IOC_JTAG_RELEASE   _IO(CSI_IOC_MAGIC, 0x11)
#define CSI_IOC_JTAG_REG_WRITE _IOW(CSI_IOC_MAGIC, 0x12, struct csi_jtag_reg)

constexpr uint8_t kFpgaSpi = 0x42;
constexpr uint8_t kFpgaDisableTx = 0x23;
constexpr uint16_t kSpiMain10PaOn = 0x2801;
constexpr uint16_t kSpiMain10PaOff = 0x2800;

std::string find_jtag_cmd() {
  if (::access("/usr/bin/quadrf-jtag", X_OK) == 0) return "/usr/bin/quadrf-jtag";
  if (::access("/home/dietpi/fpga/jtag", X_OK) == 0) return "/home/dietpi/fpga/jtag";
  return "quadrf-jtag";
}

}  // namespace

WifiRadio::WifiRadio(RadioConfig cfg) : cfg_(cfg) {
  cfg_.tx_gain = std::min(25.0, std::max(0.0, cfg_.tx_gain));
}

WifiRadio::~WifiRadio() {
  close();
}

void WifiRadio::open_csi_device() {
  if (csi_fd_ >= 0) return;
  csi_fd_ = ::open("/dev/csi_stream0", O_RDWR);
  if (csi_fd_ >= 0) {
    if (::ioctl(csi_fd_, CSI_IOC_JTAG_SETUP) != 0) {
      std::cerr << "WifiRadio: CSI_IOC_JTAG_SETUP warning: " << std::strerror(errno) << "\n";
    }
  }
}

bool WifiRadio::write_fpga_reg(uint8_t addr, uint16_t val) {
  std::lock_guard<std::mutex> lock(reg_mu_);
  if (csi_fd_ >= 0) {
    csi_jtag_reg r{};
    r.addr = addr;
    r.value = val;
    if (::ioctl(csi_fd_, CSI_IOC_JTAG_REG_WRITE, &r) == 0) {
      return true;
    }
  }

  // Fallback to JTAG command
  char cmd[128];
  std::snprintf(cmd, sizeof(cmd), "%s write 0x%02x 0x%04x >/dev/null 2>&1",
                find_jtag_cmd().c_str(), addr, val);
  return std::system(cmd) == 0;
}

void WifiRadio::pa_mute() {
  write_fpga_reg(kFpgaSpi, kSpiMain10PaOff);
  write_fpga_reg(kFpgaDisableTx, 0x0001);
}

void WifiRadio::pa_unmute() {
  write_fpga_reg(kFpgaDisableTx, 0x0000);
  write_fpga_reg(kFpgaSpi, kSpiMain10PaOn);
  if (cfg_.pa_settle_us > 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(cfg_.pa_settle_us));
  }
}

void WifiRadio::configure_rf_hardware() {
  const std::string jtag = find_jtag_cmd();
  std::ostringstream rx_spec, tx_spec;

  rx_spec << jtag << " --rx antennas=" << cfg_.rx_antenna_mask
          << ",interleave=0,tone_en=0,autosteer=0,p1=0,p2=0,p3=0,p4=0,pol="
          << cfg_.pol << ",freq=" << cfg_.freq_mhz
          << ",gain=" << static_cast<int>(cfg_.rx_gain) << ",bw=20 >/dev/null 2>&1";

  tx_spec << jtag << " --tx antennas=" << cfg_.tx_antenna_mask
          << ",tone_en=0,tx_follow_rx=0,p1=0,p2=0,p3=0,p4=0,freq="
          << cfg_.freq_mhz << ",gain=" << static_cast<int>(cfg_.tx_gain)
          << ",bw=20 >/dev/null 2>&1";

  std::cerr << "WifiRadio: tuning RX LO to " << cfg_.freq_mhz << " MHz (gain="
            << cfg_.rx_gain << " dB, ant=" << cfg_.rx_antenna_mask << ", pol=" << cfg_.pol << ")...\n";
  std::cerr << "WifiRadio: tuning TX LO to " << cfg_.freq_mhz << " MHz (gain="
            << cfg_.tx_gain << " dB, ant=" << cfg_.tx_antenna_mask << ")...\n";
  std::system(rx_spec.str().c_str());
  std::system(tx_spec.str().c_str());

  if (cfg_.enable_pa_mute) {
    pa_mute();
  } else {
    pa_unmute();
  }
}

bool WifiRadio::init() {
  close();
  open_csi_device();
  configure_rf_hardware();

  std::cerr << "WifiRadio: initializing SoapySDR (driver=mipi)...\n";
  SoapySDR::Kwargs args{{"driver", "mipi"}};
  try {
    sdr_ = SoapySDR::Device::make(args);
    if (!sdr_) {
      std::cerr << "WifiRadio: SoapySDR::Device::make failed\n";
      return false;
    }

    sdr_->setSampleRate(SOAPY_SDR_RX, 0, 20e6);
    sdr_->setSampleRate(SOAPY_SDR_TX, 0, 20e6);

    rx_stream_ = sdr_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS8, {0});
    tx_stream_ = sdr_->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS8, {0});

    sdr_->activateStream(rx_stream_);
    sdr_->activateStream(tx_stream_);

    // Re-verify RX gain register 0x6A and PA state after SoapySDR initialization
    set_rx_gain(cfg_.rx_gain);
    if (cfg_.enable_pa_mute) {
      pa_mute();
    } else {
      pa_unmute();
    }

    ready_ = true;
    std::cerr << "WifiRadio: full-duplex streams active at 20 MS/s\n";
    return true;
  } catch (const std::exception& e) {
    std::cerr << "WifiRadio init exception: " << e.what() << "\n";
    close();
    return false;
  }
}

void WifiRadio::close() {
  ready_ = false;
  if (sdr_) {
    if (rx_stream_) {
      sdr_->deactivateStream(rx_stream_);
      sdr_->closeStream(rx_stream_);
      rx_stream_ = nullptr;
    }
    if (tx_stream_) {
      sdr_->deactivateStream(tx_stream_);
      sdr_->closeStream(tx_stream_);
      tx_stream_ = nullptr;
    }
    SoapySDR::Device::unmake(sdr_);
    sdr_ = nullptr;
  }
  if (csi_fd_ >= 0) {
    ::ioctl(csi_fd_, CSI_IOC_JTAG_RELEASE);
    ::close(csi_fd_);
    csi_fd_ = -1;
  }
}

void WifiRadio::set_freq(double freq_mhz) {
  cfg_.freq_mhz = freq_mhz;
  configure_rf_hardware();
}

void WifiRadio::set_rx_gain(double gain_db) {
  cfg_.rx_gain = gain_db;
  write_fpga_reg(0x6A, static_cast<uint16_t>(std::clamp(static_cast<int>(gain_db), 0, 63)));
}

void WifiRadio::set_tx_gain(double gain_db) {
  cfg_.tx_gain = std::min(25.0, std::max(0.0, gain_db));
  const std::string jtag = find_jtag_cmd();
  char cmd[128];
  std::snprintf(cmd, sizeof(cmd), "%s --tx gain=%d >/dev/null 2>&1", jtag.c_str(), static_cast<int>(cfg_.tx_gain));
  std::system(cmd);
}

int WifiRadio::read_rx(c32* out_cf32, size_t num_samples, int timeout_us) {
  if (!ready_ || !sdr_ || !rx_stream_ || num_samples == 0) return -1;

  if (rx_cs8_scratch_.size() < num_samples * 2) {
    rx_cs8_scratch_.resize(num_samples * 2);
  }

  void* buffs[1] = {rx_cs8_scratch_.data()};
  int flags = 0;
  long long timeNs = 0;

  int ret = sdr_->readStream(rx_stream_, buffs, num_samples, flags, timeNs, timeout_us);
  if (ret <= 0) return ret;

  float pwr_acc = 0.f;
  constexpr float dc_alpha = 0.001f;
  for (int i = 0; i < ret; ++i) {
    const float r = static_cast<float>(rx_cs8_scratch_[static_cast<size_t>(2 * i)]) / 127.f;
    const float im = static_cast<float>(rx_cs8_scratch_[static_cast<size_t>(2 * i + 1)]) / 127.f;
    c32 s(r, im);
    dc_mean_ += dc_alpha * (s - dc_mean_);
    s -= dc_mean_;
    out_cf32[i] = s;
    pwr_acc += std::norm(s);
  }

  const float mean_pwr = pwr_acc / static_cast<float>(ret);
  last_energy_.store(mean_pwr, std::memory_order_relaxed);

  return ret;
}

int WifiRadio::write_tx_cs8(const int8_t* in_cs8, size_t num_samples, int timeout_us) {
  if (!ready_ || !sdr_ || !tx_stream_ || num_samples == 0) return -1;

  if (cfg_.enable_pa_mute) {
    pa_unmute();
  }

  size_t off = 0;
  while (off < num_samples) {
    const void* buffs[1] = {in_cs8 + off * 2};
    const size_t to_write = std::min<size_t>(num_samples - off, 16384);
    int flags = (off + to_write >= num_samples) ? SOAPY_SDR_END_BURST : 0;
    long long timeNs = 0;
    int ret = sdr_->writeStream(tx_stream_, buffs, to_write, flags, timeNs, timeout_us);
    if (ret > 0) {
      off += static_cast<size_t>(ret);
    } else if (ret == SOAPY_SDR_TIMEOUT) {
      continue;
    } else {
      break;
    }
  }

  if (cfg_.enable_pa_mute) {
    // Wait 22 ms for the single DSI DRM frame (19.34 ms scanout at 51.7 Hz) to finish radiating
    // over the air before asserting PA mute. Prevents repeated frame scanouts.
    std::this_thread::sleep_for(std::chrono::milliseconds(22));
    pa_mute();
  }

  return static_cast<int>(off);
}

bool WifiRadio::is_channel_clear(float* out_energy) {
  const float e = last_energy_.load(std::memory_order_relaxed);
  if (out_energy) *out_energy = e;
  return e < cfg_.cca_threshold;
}

}  // namespace quadrf_wifi
