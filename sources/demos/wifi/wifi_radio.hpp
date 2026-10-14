#pragma once

#include "wifi_constants.hpp"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

namespace quadrf_wifi {

struct RadioConfig {
  double freq_mhz = 5860.0;
  double rx_gain = 30.0;
  double tx_gain = 34.0;  // clamped to 0..40 dB
  std::string pol = "rhcp";
  int rx_antenna_mask = 0x01;
  int tx_antenna_mask = 0x01;
  bool enable_pa_mute = true;  // mute PA while listening
  bool skip_lo_retune = false;  // preserve LO and front-end configured by parent process (e.g. PhaseGaze)
  int pa_settle_us = 100;
  float cca_threshold = 0.0008f;
};

class WifiRadio {
 public:
  explicit WifiRadio(RadioConfig cfg = {});
  ~WifiRadio();

  WifiRadio(const WifiRadio&) = delete;
  WifiRadio& operator=(const WifiRadio&) = delete;

  bool init();
  void close();

  bool write_fpga_reg(uint8_t addr, uint16_t val);
  uint16_t read_fpga_reg(uint8_t addr);
  bool spi_tx(unsigned reg, unsigned d10);
  void pa_mute();
  void pa_unmute();

  void set_freq(double freq_mhz);
  void set_rx_gain(double gain_db);
  void set_tx_gain(double gain_db);
  void set_tx_elements(int mask);
  void set_rx_elements(int mask);
  void set_pa_mute(bool enable);

  int read_rx(c32* out_cf32, size_t num_samples, int timeout_us = 100000);
  int write_tx_cs8(const int8_t* in_cs8, size_t num_samples, int timeout_us = 100000);
  bool is_channel_clear(float* out_energy = nullptr);
  float get_current_energy() const { return last_energy_.load(std::memory_order_relaxed); }

  const RadioConfig& config() const { return cfg_; }
  bool is_ready() const { return ready_; }
  bool is_transmitting() const { return tx_active_.load(std::memory_order_acquire); }

 private:
  RadioConfig cfg_;
  bool ready_ = false;

  int csi_fd_ = -1;
  std::mutex reg_mu_;

  SoapySDR::Device* sdr_ = nullptr;
  SoapySDR::Stream* rx_stream_ = nullptr;
  SoapySDR::Stream* tx_stream_ = nullptr;

  std::vector<c32> rx_cf32_40m_scratch_;
  c32 hb_hist_[14]{};
  std::atomic<bool> tx_active_{false};
  std::atomic<int64_t> last_tx_end_ms_{0};
  std::atomic<float> last_energy_{0.f};
  c32 dc_mean_{0.f, 0.f};

  void open_csi_device();
  void configure_rf_hardware();
};

}  // namespace quadrf_wifi
