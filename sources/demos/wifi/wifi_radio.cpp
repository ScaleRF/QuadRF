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
  cfg_.tx_gain = std::min(40.0, std::max(0.0, cfg_.tx_gain));
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
  if (sdr_) {
    try {
      sdr_->writeRegister("FPGA", addr, val);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "write_fpga_reg(0x" << std::hex << (int)addr << ", 0x" << val << ") exc: " << e.what() << "\n";
    }
  }

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

uint16_t WifiRadio::read_fpga_reg(uint8_t addr) {
  std::lock_guard<std::mutex> lock(reg_mu_);
  if (sdr_) {
    try {
      return static_cast<uint16_t>(sdr_->readRegister("FPGA", addr));
    } catch (const std::exception& e) {
      std::cerr << "read_fpga_reg(0x" << std::hex << (int)addr << ") exc: " << e.what() << "\n";
    }
  }
  return 0xFFFF;
}

bool WifiRadio::spi_tx(unsigned reg, unsigned d10) {
  return write_fpga_reg(kFpgaSpi, static_cast<uint16_t>(((reg & 0x3F) << 10) | (d10 & 0x3FF)));
}

void WifiRadio::pa_mute() {
  write_fpga_reg(kFpgaSpi, kSpiMain10PaOff);
  write_fpga_reg(kFpgaDisableTx, 0x0001);
  write_fpga_reg(0x02, 0x0000);
  spi_tx(0, 0x00C);  // E_TX=0, MODE=TX (011), BW=20 MHz (D1=0)
}

void WifiRadio::pa_unmute() {
  write_fpga_reg(kFpgaDisableTx, 0x0000);
  spi_tx(0, 0x00C | ((cfg_.tx_antenna_mask & 0xF) << 5));
  write_fpga_reg(0x02, static_cast<uint16_t>(cfg_.tx_antenna_mask & 0xF));
  write_fpga_reg(kFpgaSpi, kSpiMain10PaOn);
  if (cfg_.pa_settle_us > 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(cfg_.pa_settle_us));
  }
}

void WifiRadio::configure_rf_hardware() {
  if (cfg_.skip_lo_retune) {
    std::cerr << "WifiRadio: skip_lo_retune enabled, preserving active LO and RF configuration\n";
    if (cfg_.enable_pa_mute) {
      pa_mute();
    } else {
      pa_unmute();
    }
    return;
  }

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

  // Set FPGA Sinc^3 CIC decimation parameter k=9 (26.7 MHz two-sided digital passband)
  // while keeping MAX2851 analog filter at 20 MHz so outer OFDM subcarriers (+/-8.125 MHz)
  // suffer minimal CIC passband droop before our 40->20 Msps half-band FIR filter.
  write_fpga_reg(0x27, 9);

  pa_mute();
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

    // Stream at 40 MS/s (2x oversampled relative to 20 MS/s 802.11a baseband)
    // and perform anti-aliasing half-band FIR filtering between 40 MS/s and 20 MS/s.
    sdr_->setSampleRate(SOAPY_SDR_RX, 0, 40e6);
    sdr_->setSampleRate(SOAPY_SDR_TX, 0, 40e6);

    rx_stream_ = sdr_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, {0});
    tx_stream_ = sdr_->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS8, {0});

    sdr_->activateStream(rx_stream_);
    sdr_->activateStream(tx_stream_);

    // Re-verify RX gain register 0x6A, CIC k=9, and PA state after SoapySDR initialization
    write_fpga_reg(0x27, 9);
    set_rx_gain(cfg_.rx_gain);
    if (cfg_.enable_pa_mute) {
      pa_mute();
    } else {
      pa_unmute();
    }

    ready_ = true;
    std::cerr << "WifiRadio: full-duplex streams active (40 MS/s oversampled -> 20 MS/s baseband), reg 0x23=0x"
              << std::hex << read_fpga_reg(0x23) << std::dec << "\n";
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

void WifiRadio::set_pa_mute(bool enable) {
  cfg_.enable_pa_mute = enable;
  if (enable) pa_mute();
  else pa_unmute();
}

void WifiRadio::set_rx_gain(double gain_db) {
  cfg_.rx_gain = gain_db;
  write_fpga_reg(0x6A, static_cast<uint16_t>(std::clamp(static_cast<int>(gain_db), 0, 63)));
}

void WifiRadio::set_tx_gain(double gain_db) {
  cfg_.tx_gain = std::min(40.0, std::max(0.0, gain_db));
  const int g = static_cast<int>(cfg_.tx_gain);
  spi_tx(9, (g << 4) | 0xF);
}

void WifiRadio::set_tx_elements(int mask) {
  cfg_.tx_antenna_mask = mask & 0xF;
  if (!cfg_.enable_pa_mute || tx_active_.load(std::memory_order_acquire)) {
    write_fpga_reg(0x02, static_cast<uint16_t>(cfg_.tx_antenna_mask));
    spi_tx(0, 0x00C | ((cfg_.tx_antenna_mask & 0xF) << 5));
  }
}

void WifiRadio::set_rx_elements(int mask) {
  cfg_.rx_antenna_mask = mask & 0xF;
  const unsigned m = static_cast<unsigned>(cfg_.rx_antenna_mask & 0x1F);
  // Program MAX2851 Main6 (E_RX / E_LNA bits) via FPGA SPI bridge 0x43 and restrobe 0x6A
  write_fpga_reg(0x43, static_cast<uint16_t>((6u << 10) | ((m << 5) | m)));
  set_rx_gain(cfg_.rx_gain);
}

int WifiRadio::read_rx(c32* out_cf32, size_t num_samples, int timeout_us) {
  if (!ready_ || !sdr_ || !rx_stream_ || num_samples == 0) return -1;

  const size_t n40 = num_samples * 2;
  if (rx_cf32_40m_scratch_.size() < 14 + n40) {
    rx_cf32_40m_scratch_.resize(14 + n40);
  }
  std::memcpy(rx_cf32_40m_scratch_.data(), hb_hist_, 14 * sizeof(c32));

  void* buffs[1] = {rx_cf32_40m_scratch_.data() + 14};
  int flags = 0;
  long long timeNs = 0;

  int ret40 = sdr_->readStream(rx_stream_, buffs, n40, flags, timeNs, timeout_us);
  if (ret40 <= 1) return (ret40 < 0) ? ret40 : 0;

  const int ret20 = ret40 / 2;
  const int consumed40 = ret20 * 2;
  std::memcpy(hb_hist_, rx_cf32_40m_scratch_.data() + consumed40, 14 * sizeof(c32));

  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  const bool self_tx_blank =
      cfg_.enable_pa_mute &&
      (tx_active_.load(std::memory_order_acquire) ||
       (now_ms - last_tx_end_ms_.load(std::memory_order_acquire) < 6));

  if (self_tx_blank) {
    last_energy_.store(0.f, std::memory_order_relaxed);
    std::fill(out_cf32, out_cf32 + ret20, c32(0.f, 0.f));
    return ret20;
  }

  // 15-tap symmetric half-band FIR decimator (40 MS/s -> 20 MS/s):
  // Passband [-8.125, +8.125] MHz ripple < 0.02 dB; stopband [11.875, 20.0] MHz > 38 dB attenuation.
  constexpr float h0 = 0.5000000f;
  constexpr float h1 = 0.3134022f;
  constexpr float h3 = -0.0921536f;
  constexpr float h5 = 0.0363834f;
  constexpr float h7 = -0.0076320f;

  float pwr_acc = 0.f;
  constexpr float dc_alpha = 0.0001f;
  const c32* x = rx_cf32_40m_scratch_.data() + 7;  // center tap aligned at index 7 + 2*i

  for (int i = 0; i < ret20; ++i) {
    const int idx = 2 * i;
    c32 s = h0 * x[idx] +
            h1 * (x[idx - 1] + x[idx + 1]) +
            h3 * (x[idx - 3] + x[idx + 3]) +
            h5 * (x[idx - 5] + x[idx + 5]) +
            h7 * (x[idx - 7] + x[idx + 7]);

    if (std::norm(s - dc_mean_) < 0.002f) {
      dc_mean_ += dc_alpha * (s - dc_mean_);
    }
    s -= dc_mean_;
    out_cf32[i] = s;
    pwr_acc += std::norm(s);
  }

  const float mean_pwr = pwr_acc / static_cast<float>(ret20);
  last_energy_.store(mean_pwr, std::memory_order_relaxed);

  return ret20;
}

int WifiRadio::write_tx_cs8(const int8_t* in_cs8, size_t num_samples, int timeout_us) {
  if (!ready_ || !sdr_ || !tx_stream_ || num_samples == 0) return -1;

  tx_active_.store(true, std::memory_order_release);

  size_t off = 0;
  while (off < num_samples) {
    const void* buffs[1] = {in_cs8 + off * 2};
    const size_t to_write = std::min<size_t>(num_samples - off, 131072);
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
    // Allow 3 ms for the kernel flip thread (dsi_stream_blit_to_fb) to copy the 3.3 MB
    // staging buffer into the DRM framebuffer before unmuting the PA, then hold unmute
    // for 28 ms (1.45 VSYNC periods at 51.7 Hz) so the burst radiates cleanly.
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    pa_unmute();
    std::this_thread::sleep_for(std::chrono::milliseconds(28));
    pa_mute();
  }

  // Queue a 64-sample zero-tail frame immediately after muting the external PA, and keep
  // tx_active_ true for 21 ms (2 ms blit + 19.34 ms VSYNC period) so board-level MAX2850->MAX2851
  // leakage during the final unflipped scanout is completely blanked by read_rx().
  alignas(16) static constexpr int8_t zero_samples[128] = {0};
  const void* zero_buff[1] = {zero_samples};
  int end_flags = SOAPY_SDR_END_BURST;
  sdr_->writeStream(tx_stream_, zero_buff, 64, end_flags, 0, timeout_us);
  std::this_thread::sleep_for(std::chrono::milliseconds(21));

  const auto end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  last_tx_end_ms_.store(end_ms, std::memory_order_release);
  tx_active_.store(false, std::memory_order_release);

  return static_cast<int>(off);
}

bool WifiRadio::is_channel_clear(float* out_energy) {
  const float e = last_energy_.load(std::memory_order_relaxed);
  if (out_energy) *out_energy = e;
  return e < cfg_.cca_threshold;
}

}  // namespace quadrf_wifi
