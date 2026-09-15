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
  double freq_mhz = 5805.0;
  double rx_gain = 8.0;
  double tx_gain = 0.0;  // clamped to 0..25 dB
  std::string pol = "lhcp";
  int rx_antenna_mask = 0x0F;
  int tx_antenna_mask = 0x01;
  bool enable_pa_mute = true;  // mute PA while listening
  int pa_settle_us = 100;
  float cca_threshold = 0.05f;
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
  void pa_mute();
  void pa_unmute();

  void set_freq(double freq_mhz);
  void set_rx_gain(double gain_db);
  void set_tx_gain(double gain_db);
  void set_pa_mute(bool enable) { cfg_.enable_pa_mute = enable; }

  int read_rx(c32* out_cf32, size_t num_samples, int timeout_us = 100000);
  int write_tx_cs8(const int8_t* in_cs8, size_t num_samples, int timeout_us = 100000);
  bool is_channel_clear(float* out_energy = nullptr);
  float get_current_energy() const { return last_energy_.load(std::memory_order_relaxed); }

  const RadioConfig& config() const { return cfg_; }
  bool is_ready() const { return ready_; }

 private:
  RadioConfig cfg_;
  bool ready_ = false;

  int csi_fd_ = -1;
  std::mutex reg_mu_;

  SoapySDR::Device* sdr_ = nullptr;
  SoapySDR::Stream* rx_stream_ = nullptr;
  SoapySDR::Stream* tx_stream_ = nullptr;

  std::vector<int8_t> rx_cs8_scratch_;
  std::atomic<float> last_energy_{0.f};
  c32 dc_mean_{0.f, 0.f};

  void open_csi_device();
  void configure_rf_hardware();
};

}  // namespace quadrf_wifi
