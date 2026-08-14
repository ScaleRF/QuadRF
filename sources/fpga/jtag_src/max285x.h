// max285x.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX2851_REG_ADDR 0x43  // RX chip (SPI forwarded via FPGA reg 0x43)
#define MAX2850_REG_ADDR 0x42  // TX chip (SPI forwarded via FPGA reg 0x42)

/* Provided by jtag.c */
int jtag_write_u16(int fd, uint8_t addr, uint16_t value);
int jtag_read_u16 (int fd, uint8_t addr, uint16_t *value_out);

/* Existing functions (already implemented in your max285x.c) */
int max2851_init(int fd);
int max2850_init(int fd);

int max2851_set_freq_mhz(int fd, double mhz);
int max2850_set_freq_mhz(int fd, double mhz);

int max2851_set_rx_gain(int fd, int16_t rx_gain_setting);
int max2850_set_tx_gain(int fd, uint16_t tx_gain_setting);

/* New wrappers matching your current jtag.c intent */
int max2850_set_idle(int fd, int bw_mhz);
int max2851_set_idle(int fd, int bw_mhz);

int max2850_tx_on(int fd,
                  int bw_mhz,
                  bool set_freq, double freq_mhz,
                  bool set_gain, uint16_t gain,
                  bool set_antennas, uint16_t antennas_mask);
int max2850_tx_off(int fd, int bw_mhz);



int max2851_rx_on(int fd,
                  int bw_mhz,
                  bool set_freq, double freq_mhz,
                  bool set_gain, int16_t gain,
                  bool set_antennas, uint8_t rx_mask);
int max2851_rx_off(int fd, int bw_mhz /* 20 or 40 (reg0 contains BW) */);

/* Status dump functions */
int max2850_status(int fd);
int max2851_status(int fd);

#ifdef __cplusplus
}
#endif