// max285x.c
#include "max285x.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * This file forwards MAX2850/MAX2851 SPI "words" through FPGA JTAG registers:
 * MAX2850_REG_ADDR (0x42) for TX chip
 * MAX2851_REG_ADDR (0x43) for RX chip
 *
 * Word format:
 * [15:10] register address
 * [9:0]   register data (10-bit)
 */

// ---------------------------
// Static base registers
// ---------------------------
static const uint16_t max2851_base_regs[32] = {
    /* 0: */ 0x00A, // Main0: MODE[2:0] (D4:D2), RFBW (D1) (D1=0 => 40 MHz, D1=1 => 20 MHz)
    /* 1: */ 0x000, // Main1: LNA_GAIN[2:0] (D7:D5) + VGA_GAIN[4:0] (D4:D0) (Rx gain word)
    /* 2: */ 0x1C0, // Main2: LNA_BAND[1:0] (D6:D5) selects RF band; 01=~5.2–5.5GHz (datasheet default)
    /* 3: */ 0x000, // Main3: Temperature sensor control: TS_EN (D7), TS_TRIG (D6); TS_READ[4:0] readback
    /* 4: */ 0x31C, // Main4: RESERVED
    /* 5: */ 0x000, // Main5: RSSI_MUX_SEL[2:0] (D8:D6), RSSI_RX_SEL[2:0] (D5:D3), RXHP (D1)
    /* 6: */ 0x3FF, // Main6: RX_GAIN_PROG_SEL[5:1] (D9:D5) + E_RX[5:1] (D4:D0) (Rx channel enable/program mask)
    /* 7: */ 0x024, // Main7: RESERVED
    /* 8: */ 0x000, // Main8: W/R, no named fields in summary (write 0 unless you have a reason)
    /* 9: */ 0x00F, // Main9: TX_GAIN[5:0] (D9:D4); D3:D0 reserved (set to default)
    /* 10: */ 0x000, // Main10: RESERVED
    /* 11: */ 0x060, // Main11: RESERVED
    /* 12: */ 0x000, // Main12: SKIPPED
    /* 13: */ 0x000, // Main13: RESERVED (keep datasheet default/typical)
    /* 14: */ 0x160, // Main14: E_CLKOUT2 (D9=1 enables CLKOUT2), DOUT_SEL (D1=0 => PLL lock detect on DOUT)
    /* 15: */ 0x246, // Main15: Synth integer divide N (SYN_CONFIG_N[6:0]=D6:D0), plus VAS_TRIG_EN (D9)
    /* 16: */ 0x000, // Main16: Synth fractional divide F[19:10] MSBs (D9:D0)
    /* 17: */ 0x000, // Main17: Synth fractional divide F[9:0] LSBs (D9:D0)
    /* 18: */ 0x080, // Main18: XTAL_TUNE[7:0] crystal oscillator frequency trim (0x80 = datasheet default)
    /* 19: */ 0x0DF, // Main19: VAS_RELOCK_SEL (D7), VAS_MODE (D6), VAS_SPI[5:0] (D5:D0) (VCO sub-band auto-select controls)
    /* 20: */ 0x1EA, // Main20: RESERVED (keep datasheet default/typical)
    /* 21: */ 0x0BF, // Main21: DIE_ID[2:0] readback (active when DIE_ID_READ=1 in Main27)
    /* 22: */ 0x1FE, // Main22: RESERVED 
    /* 23: */ 0x065, // Main23: RESERVED 
    /* 24: */ 0x14B, // Main24: RESERVED 
    /* 25: */ 0x3A8, // Main25: RESERVED 
    /* 26: */ 0x015, // Main26: RESERVED 
    /* 27: */ 0x180, // Main27: DIE_ID_READ (D9) and VAS_VCO_READ (D5) control which readbacks appear in Main21/Main19
    /* 28: */ 0x061, // Main28: PA_BIAS_DLY[3:0] (PA_BIAS turn-on delay; 0x3 => 1.0us default; other bits reserved)
    /* 29: */ 0x000, // Main29: RESERVED
    /* 30: */ 0x000, // Main30: RESERVED
    /* 31: */ 0x000, // Main31: RESERVED
};

static const uint16_t max2850_base_regs[32] = {
    /* 0: */ 0x00E, // Main0: E_TX[4:1] (D8:D5) enables TX channels; MODE[2:0] (D4:D2); RFBW (D1); M/L_SEL (D0=0)
    /* 1: */ 0x000, // Main1: LNA_GAIN[2:0] (not used for MAX2850)
    /* 2: */ 0x1C0, // Main2: LNA_BAND[1:0] (not used for MAX2850)
    /* 3: */ 0x000, // Main3: Temperature sensor control: TS_EN (D7), TS_TRIG (D6); TS_READ[4:0] readback
    /* 4: */ 0x31C, // Main4: RESERVED (keep datasheet default/typical)
    /* 5: */ 0x000, // Main5: RSSI_MUX_SEL[2:0] (D8:D6) selects RSSI output source; RXHP (D1) controls Rx VGA HP corner during gain adjust
    /* 6: */ 0x3EB, // Main6: RESERVED in datasheet summary (keep datasheet default/typical)
    /* 7: */ 0x024, // Main7: RESERVED (keep datasheet default/typical)
    /* 8: */ 0x000, // Main8: W/R, no named fields in summary (write 0 unless you have a reason)
    /* 9: */ 0x00F, // Main9: TX_GAIN[5:0] (D9:D4) + TX_GAIN_PROG_SEL[4:1] (D3:D0) selects which TX channel(s) receive that gain
    /* 10: */ 0x001, // Main10: Reserved, but from eval kit a 1 needed for PA bias out = 2.85V
    /* 11: */ 0x060, // Main11: E_TX_AMD[1:0] (D3:D2) selects Tx calibration AM detector channel; PA_DET_SEL[1:0] (D1:D0) selects PA_DET mux output
    /* 12: */ 0x000, // Main12: SKIPPED
    /* 13: */ 0x000, // Main13: RESERVED
    /* 14: */ 0x160, // Main14: DOUT_SEL (D1=0 => PLL lock detect on DOUT; D1=1 => SPI readback). Other bits reserved; keep default.
    /* 15: */ 0x246, // Main15: VAS_TRIG_EN (D9) + SYN_CONFIG_N[6:0] (integer divide ratio N)
    /* 16: */ 0x000, // Main16: SYN_CONFIG_F[19:10] (fractional divide ratio MSBs)
    /* 17: */ 0x000, // Main17: SYN_CONFIG_F[9:0] (fractional divide ratio LSBs)
    /* 18: */ 0x080, // Main18: XTAL_TUNE[7:0] crystal oscillator frequency trim (0x80 = datasheet default)
    /* 19: */ 0x05F, // Main19: VAS_RELOCK_SEL (D7), VAS_MODE (D6), VAS_SPI[5:0] (D5:D0) (VCO sub-band auto-select controls)
    /* 20: */ 0x1EA, // Main20: RESERVED
    /* 21: */ 0x0BF, // Main21: DIE_ID[2:0] readback (active when DIE_ID_READ=1 in Main27)
    /* 22: */ 0x1FE, // Main22: RESERVED
    /* 23: */ 0x065, // Main23: RESERVED
    /* 24: */ 0x14B, // Main24: RESERVED
    /* 25: */ 0x3A8, // Main25: RESERVED
    /* 26: */ 0x015, // Main26: RESERVED
    /* 27: */ 0x180, // Main27: DIE_ID_READ (D9) and VAS_VCO_READ (D5) readback selectors (same concept as MAX2851)
    /* 28: */ 0x061, // Main28: PA_BIAS_DLY[3:0] PA_BIAS turn-on delay (0x3 => 1.0us default)
    /* 29: */ 0x000, // Main29: RESERVED
    /* 30: */ 0x000, // Main30: RESERVED
    /* 31: */ 0x000, // Main31: RESERVED
};

static uint16_t max2851_lna_band_reg2_for_freq(double mhz)
{
    /* Main2: LNA_BAND[1:0] at D[6:5]; keep reserved bits at default. */
    if (mhz < 5200.0) return 0x180;  /* 00: 4.9–5.2 GHz */
    if (mhz < 5500.0) return 0x1A0;  /* 01: 5.2–5.5 GHz (default) */
    if (mhz < 5800.0) return 0x1C0;  /* 10: 5.5–5.8 GHz */
    return 0x1E0;                    /* 11: 5.8–5.9 GHz */
}

static uint16_t main0_force_mode_bw(uint16_t reg0_template, uint16_t mode_bits, int bw_mhz)
{
    /* Main0: D[4:2]=MODE, D1=RFBW. Preserve other bits from template. */
    uint16_t r = (uint16_t)(reg0_template & 0x3FFu);

    /* Clear MODE (bits 4:2) and insert desired mode bits (already shifted). */
    r &= (uint16_t)~(0x7u << 2);
    r |= (uint16_t)(mode_bits & (0x7u << 2));

    /* Force BW bit. */
    r &= (uint16_t)~(1u << 1);
    if (bw_mhz == 40) r |= (uint16_t)(1u << 1);

    return r;
}


// Read / Write MAX285x SPI registers (using the FPGA as a tunnel)

static int max2850_word(int fd, uint16_t reg, uint16_t data10)
{
    uint16_t w = (uint16_t)(((reg & 0x3Fu) << 10) | (data10 & 0x3FFu));
    return jtag_write_u16(fd, MAX2850_REG_ADDR, w);
}

static int max2851_word(int fd, uint16_t reg, uint16_t data10)
{
    uint16_t w = (uint16_t)(((reg & 0x3Fu) << 10) | (data10 & 0x3FFu));
    return jtag_write_u16(fd, MAX2851_REG_ADDR, w);
}

// Readback only gives the PLL lock state which is what output mux is set to
static int max2851_readback(int fd, uint16_t *out_value)
{
    return jtag_read_u16(fd, (uint8_t)MAX2851_REG_ADDR, out_value);
}

// Readback only gives the PLL lock state which is what output mux is set to
static int max2850_readback(int fd, uint16_t *out_value)
{
    return jtag_read_u16(fd, (uint8_t)MAX2850_REG_ADDR, out_value);
}


// ---------------------------
// Ported high-level functions
// ---------------------------

int max2851_init(int fd)
{
    uint16_t readback = 0;
    for (int k = 0; k < 32; k++) {
        if (k == 12) continue; // skipped in reg-map
        if (max2851_word(fd, (uint16_t)k, (uint16_t)max2851_base_regs[k]) != 0) return -1;
        if (max2851_readback(fd, &readback) != 0) return -1;
    }

    if (readback != 0xFFFFu) {
        fprintf(stderr, "(!!! Rx Failed Lock !!!) readback=0x%04X\n", readback);
    }

    // Now write local registers: set local register programming bit via reg0.
    if (max2851_word(fd, 0, (uint16_t)(max2851_base_regs[0] | 0x01)) != 0) return -1;

    int16_t max2851_local_reg[32] = {
    /* Local regs are selected by setting Main0.M/L_SEL=1 (see below). Most local addresses are RESERVED. */
    /* 0: */ 0x000, // Local0: skipped (no definition in reg-map)
    /* 1: */ 0x000, 
    /* 2: */ 0x000, 
    /* 3: */ 0x000, 
    /* 4: */ 0x380, // Local4: RFDET_MUX_SEL[2:0] (D9:D7) RF RSSI channel select; 0b111 = "Not selected" (default)
    /* 5: */ 0x000, 
    /* 6: */ 0x000, 
    /* 7: */ 0x000, 
    /* 8: */ 0x1AA, // Unknown, RESERVED
    /* 9: */ 0x114, // 
    /* 10: */ 0x354, // 
    /* 11: */ 0x073, // 
    /* 12: */ 0x000, // 
    /* 13: */ 0x000, // 
    /* 14: */ 0x000, // 
    /* 15: */ 0x000, // 
    /* 16: */ 0x000, // 
    /* 17: */ 0x000, // 
    /* 18: */ 0x000, // 
    /* 19: */ 0x000, // 
    /* 20: */ 0x000, // 
    /* 21: */ 0x000, // 
    /* 22: */ 0x000, // 
    /* 23: */ 0x000, // 
    /* 24: */ 0x1CC, // 
    /* 25: */ 0x12B, // 
    /* 26: */ 0x165, // 
    /* 27: */ 0x002, // Local27: TX_AMD_BB_GAIN (D2) + TX_AMD_RF_GAIN (D1:D0) (Tx calibration AM-detector gain trims)
    /* 28: */ 0x004, // 
    /* 29: */ 0x000, // 
    /* 30: */ 0x000, // 
    /* 31: */ 0x200, // Charge pump trim
};

    for (int k = 0; k < 32; k++) {
        if (k == 0 || k == 29 || k == 30) continue; // skipped in reg-map
        if (max2851_word(fd, (uint16_t)k, (uint16_t)max2851_local_reg[k]) != 0) return -1;
        if (max2851_readback(fd, &readback) != 0) return -1;
        fputc('*', stdout);
        fflush(stdout);
    }
    fputc('\n', stdout);

    if (readback != 0xFFFFu) {
        fprintf(stderr, "(!!! Rx Failed Lock (local) !!!) readback=0x%04X\n", readback);
    }

    // End with main register programming (clear local bit).
    if (max2851_word(fd, 0, (uint16_t)(max2851_base_regs[0] | 0x00)) != 0) return -1;

    return 0;
}

int max2851_set_rx_gain_db(int fd, int16_t rx_gain_setting)
{  // Handled by FPGA, which programs VGA, LNA, digital gain automatically
    return jtag_write_u16(fd, 0x6A, (uint16_t)rx_gain_setting);
}


static int max2851_write_main(int fd, uint16_t main_addr, uint16_t data10)
{
    // Main reg access: Main addr select is in Main Address 0 D0 = 0
    return max2851_word(fd, main_addr, data10 & 0x03FFu);
}

// Program Main Address 0 (mode + BW)
static int max2851_set_mode_bw(int fd, uint8_t mode, int bw_mhz)
{
    const uint16_t bw_bit = (bw_mhz == 20) ? 0u : 1u; // default 40 if caller passes anything else
    const uint16_t reg0 = (uint16_t)(((mode & 0x7u) << 2) | (bw_bit << 1) | 0u);
    return max2851_write_main(fd, 0, reg0);
}

static uint16_t max2851_reg0_mode_bw(uint8_t mode, int bw_mhz)
{
    /* reg0: MODE[2:0] in D[4:2], RFBW in D1, D0=0 for main */
    const uint16_t bw_bit = (bw_mhz == 20) ? 0u : 1u; /* default 40 */
    return (uint16_t)(((mode & 0x7u) << 2) | (bw_bit << 1) | 0u);
}


int max2851_rx_on(int fd,
                  int bw_mhz,
                  bool set_freq, double freq_mhz,
                  bool set_gain, int16_t gain,
                  bool set_antennas, uint8_t rx_mask)
{
    /* Default: Rx1..Rx4 enabled (mask=0x0F). rx_mask is 5-bit (Rx1..Rx5). */
    uint8_t m = set_antennas ? (uint8_t)(rx_mask & 0x1Fu) : 0x0Fu;
    if (m == 0) m = 0x0Fu;

    /* Program reg6: RX_GAIN_PROG_SEL (D[9:5]) and E_RX (D[4:0]) */
    const uint16_t reg6 = (uint16_t)(((uint16_t)m << 5) | (uint16_t)m);
    if (max2851_write_main(fd, 6, reg6) != 0) return -1;

    /* Enter RX mode: MODE=010 */
    if (max2851_write_main(fd, 0, max2851_reg0_mode_bw(0x2u, bw_mhz)) != 0) return -1;

    /* Program PLL if requested */
    if (set_freq) {
        if (max2851_set_freq_mhz(fd, freq_mhz) != 0) return -1;
    }

    /* Program gain if requested */
    if (set_gain) {
        if (max2851_set_rx_gain_db(fd, gain) != 0) return -1;
    }

    return 0;
}

int max2851_rx_off(int fd, int bw_mhz)
{
    // Standby mode (MODE=001). Keep BW consistent because reg0 includes BW.
    return max2851_set_mode_bw(fd, 0x1u /* STANDBY */, (bw_mhz == 20) ? 20 : 40);
}

int max2851_set_idle(int fd, int bw_mhz)
{
    if (bw_mhz != 20 && bw_mhz != 40) { errno = EINVAL; return -1; }

    /* MODE=000 (standby), BW forced */
    const uint16_t reg0 = main0_force_mode_bw((uint16_t)max2851_base_regs[0], (0u << 2), bw_mhz);
    return max2851_word(fd, 0, reg0);
}

// -----------------------------------------------------------------------------
// MAX2850 (Tx) 
// -----------------------------------------------------------------------------

int max2850_init(int fd)
{
    uint16_t readback = 0;
    for (int k = 0; k < 32; k++) {
        if (k == 12) continue; // skipped in reg-map
        if (max2850_word(fd, (uint16_t)k, (uint16_t)max2850_base_regs[k]) != 0) return -1;
        if (max2850_readback(fd, &readback) != 0) return -1;
    }

    if (readback != 0xFFFFu) {
        fprintf(stderr, "(!!! Tx Failed Lock !!!) readback=0x%04X\n", readback);
    }

    // Write local registers: set local register programming bit via reg0.
    if (max2850_word(fd, 0, (uint16_t)(max2850_base_regs[0] | 0x01)) != 0) return -1;

    int16_t max2850_local_reg[32] = {
        /* 0: */ 0x000, 
        /* 1: */ 0x000,
        /* 2: */ 0x000,
        /* 3: */ 0x000,
        /* 4: */ 0x380,
        /* 5: */ 0x000,
        /* 6: */ 0x000,
        /* 7: */ 0x000,
        /* 8: */ 0x1AA,
        /* 9: */ 0x114,
        /* 10:*/ 0x354,
        /* 11:*/ 0x073,
        /* 12:*/ 0x000,
        /* 13:*/ 0x000,
        /* 14:*/ 0x000,
        /* 15:*/ 0x000,
        /* 16:*/ 0x000,
        /* 17:*/ 0x000,
        /* 18:*/ 0x000,
        /* 19:*/ 0x000,
        /* 20:*/ 0x000,
        /* 21:*/ 0x000,
        /* 22:*/ 0x000,
        /* 23:*/ 0x000,
        /* 24:*/ 0x0C4,
        /* 25:*/ 0x12B,
        /* 26:*/ 0x165,
        /* 27:*/ 0x002,
        /* 28:*/ 0x004,
        /* 29:*/ 0x000,
        /* 30:*/ 0x000,
        /* 31:*/ 0x200
    };

    for (int k = 0; k < 32; k++) {
        if (k == 0 || k == 29 || k == 30) continue; // skipped in reg-map
        if (max2850_word(fd, (uint16_t)k, (uint16_t)max2850_local_reg[k]) != 0) return -1;
        if (max2850_readback(fd, &readback) != 0) return -1;
    }

    if (readback != 0xFFFFu) {
        fprintf(stderr, "(!!! Tx Failed Lock (local) !!!) readback=0x%04X\n", readback);
    }

    // End with main register programming (clear local bit).
    if (max2850_word(fd, 0, (uint16_t)(max2850_base_regs[0] | 0x00)) != 0) return -1;

    return 0;
}


int max2850_tx_on(int fd,
                  int bw_mhz,
                  bool set_freq, double freq_mhz,
                  bool set_gain, uint16_t gain,
                  bool set_antennas, uint16_t antennas_mask)
{
    if (bw_mhz != 20 && bw_mhz != 40) { errno = EINVAL; return -1; }

    /* --- READ LIVE STATE --- */
    // Route SPI readback to DOUT to read the MAX2850
    uint16_t orig_reg14 = max2850_base_regs[14];
    if (max2850_word(fd, 14, orig_reg14 | (1 << 1)) != 0) return -1;

    // Send read command for Register 0
    uint16_t read_cmd = 0x8000 | (0 << 10);
    jtag_write_u16(fd, MAX2850_REG_ADDR, read_cmd);

    // Fetch the result
    uint16_t current_reg0 = 0;
    jtag_read_u16(fd, MAX2850_REG_ADDR, &current_reg0);

    // Restore DOUT routing
    max2850_word(fd, 14, orig_reg14);

    /* --- MODIFY STATE --- */
    // Strip down to the 10-bit payload
    current_reg0 &= 0x3FF;

    // Clear MODE (bits 4:2) and BW (bit 1)
    current_reg0 &= (uint16_t)~((0x7u << 2) | (1u << 1));

    // Force TX Mode (011) and selected BW
    current_reg0 |= (3u << 2);
    if (bw_mhz == 40) current_reg0 |= (1u << 1);

    // If antennas are explicitly specified, update E_TX bits (D8:D5).
    // Otherwise, leave them alone to preserve hardware state!
    if (set_antennas) {
        current_reg0 &= (uint16_t)~(0xFu << 5);
        current_reg0 |= (uint16_t)((antennas_mask & 0xFu) << 5);
    }

    /* --- WRITE STATE --- */
    if (max2850_word(fd, 0, current_reg0) != 0) return -1;

    if (set_freq) {
        if (max2850_set_freq_mhz(fd, freq_mhz) != 0) return -1;
    }

    if (set_gain) {
        if (max2850_set_tx_gain(fd, gain) != 0) return -1;
    }

    /* FPGA global setting: enable delta-sigma / RF switch path for TX */
    if (jtag_write_u16(fd, 0x23, 0x0000) != 0) return -1;

    /* Enable PA Bias */
    if (max2850_word(fd, 10, 0x0001) != 0) return -1;

    /* If antennas specified, update FPGA baseband channel mask */
    if (set_antennas) {
        if (jtag_write_u16(fd, 0x02, antennas_mask) != 0) return -1;
    }

    return 0;
}

int max2850_set_tx_gain(int fd, uint16_t tx_gain_setting)
{
    /* Always program requested value (including 0) in this API. */
    if (max2850_word(fd, 9, (uint16_t)(((tx_gain_setting & 0x3Fu) << 4) | 0x000Fu)) != 0) return -1;
    return 0;
}

int max2850_set_idle(int fd, int bw_mhz)
{
    if (bw_mhz != 20 && bw_mhz != 40) { errno = EINVAL; return -1; }

    /* MODE=000 (standby), BW forced; preserve E_TX bits from template */
    const uint16_t reg0 = main0_force_mode_bw((uint16_t)max2850_base_regs[0], (0u << 2), bw_mhz);
    if (max2850_word(fd, 0, reg0) != 0) return -1;

    if (jtag_write_u16(fd, 0x23, 0x0001) != 0) return -1; // disable_tx (allows rx mux to polarization switch again)

    /* Disable PA bias when idling */
    if (max2850_word(fd, 10, 0x0000) != 0) return -1;

    /* Also clear FPGA antenna mask to ensure no TX path is enabled */
    if (jtag_write_u16(fd, 0x02, 0x0000) != 0) return -1;

    return 0;
}


static int max285x_set_freq_common(int fd, uint8_t fpga_addr, double freq_mhz)
{
    double ratio = freq_mhz / 80.0;
    long long idiv = (long long)floor(ratio);
    double frac = ratio - (double)idiv;
    long long fdiv = llround(frac * (double)(1u << 20));

    /* Handle special-case rounding carry: FDIV == 2^20 ? increment IDIV */
    if (fdiv == (1u << 20)) {
        idiv += 1;
        fdiv = 0;
    }

    const uint16_t w15 = (uint16_t)((15u << 10) | (1u << 9) | ((unsigned long long)idiv & 0x7FULL));
    const uint16_t w16 = (uint16_t)((16u << 10) | ((unsigned long long)(fdiv >> 10) & 0x3FFULL));
    const uint16_t w17 = (uint16_t)((17u << 10) | ((unsigned long long)fdiv & 0x3FFULL));

    if (jtag_write_u16(fd, fpga_addr, w15) != 0) return -1;
    if (jtag_write_u16(fd, fpga_addr, w16) != 0) return -1;
    if (jtag_write_u16(fd, fpga_addr, w17) != 0) return -1;

    /* Update LNA_BAND (Main2) to match the new RF frequency (RX path matching) */
    if((fpga_addr == MAX2851_REG_ADDR)) {
    	uint16_t reg2 = max2851_lna_band_reg2_for_freq(freq_mhz);

    	uint16_t w2 = (uint16_t)((2u << 10) | (reg2 & 0x3FF));
    	if (jtag_write_u16(fd, fpga_addr, w2) != 0) return -1;
    }

    return 0;
}

int max2850_set_bw(int fd, int bw_mhz)
{
    if (bw_mhz != 20 && bw_mhz != 40) { errno = EINVAL; return -1; }

    // No shadows: start from base template reg0 and force TX mode (MODE=011) + BW.
    const uint16_t reg0 = main0_force_mode_bw((uint16_t)max2850_base_regs[0], (3u << 2), bw_mhz); // TX=011
    return max2850_word(fd, 0, reg0);
}

int max2851_set_freq_mhz(int fd, double mhz)
{
    return max285x_set_freq_common(fd, MAX2851_REG_ADDR, mhz);
}

int max2850_set_freq_mhz(int fd, double mhz)
{
    return max285x_set_freq_common(fd, MAX2850_REG_ADDR, mhz);
}

// -----------------------------------------------------------------------------
// Status Readers
// -----------------------------------------------------------------------------

int max2850_status(int fd)
{
    uint16_t orig_reg14 = max2850_base_regs[14]; // Contains DOUT_SEL = 0
    
    // 1. Check PLL Lock first (ensure DOUT is set to lock detect)
    uint16_t pll_lock_val = 0;
    if (max2850_word(fd, 14, orig_reg14) != 0) return -1;
    max2850_readback(fd, &pll_lock_val);
    bool pll_locked = (pll_lock_val == 0xFFFFu);

    // 2. Set D1 (bit 1) of Main14 to route SPI readback to DOUT
    if (max2850_word(fd, 14, orig_reg14 | (1 << 1)) != 0) return -1;

    uint16_t regs[32] = {0};
    for (int i = 0; i < 32; i++) {
        if (i == 12) continue; // skipped in reg-map
        
        uint16_t cmd = 0x8000 | ((uint16_t)(i & 0x1F) << 10);
        jtag_write_u16(fd, MAX2850_REG_ADDR, cmd);
        jtag_read_u16(fd, MAX2850_REG_ADDR, &regs[i]);
    }

    // 3. Restore Main14 back to default
    if (max2850_word(fd, 14, orig_reg14) != 0) return -1;

    uint16_t r0 = regs[0] & 0x3FF;
    uint16_t r9 = regs[9] & 0x3FF;
    uint16_t r15 = regs[15] & 0x3FF;
    uint16_t r16 = regs[16] & 0x3FF;
    uint16_t r17 = regs[17] & 0x3FF;

    // Tx On/Off
    uint16_t mode = (r0 >> 2) & 0x7;
    bool tx_on = (mode == 3);

    // Antennas enabled
    uint16_t e_tx = (r0 >> 5) & 0xF;
    
    // LO Frequency
    uint16_t idiv = r15 & 0x7F;
    uint32_t fdiv = ((r16 & 0x3FF) << 10) | (r17 & 0x3FF);
    double ratio = (double)idiv + ((double)fdiv / (double)(1<<20));
    double freq_mhz = ratio * 80.0;

    // Gain (Main9 D9:D4)
    uint16_t tx_gain = (r9 >> 4) & 0x3F;

    printf("Tx:\n");
    printf("- PLL Lock: %s\n", pll_locked ? "LOCKED" : "UNLOCKED");
    printf("- Tx is %s\n", tx_on ? "ON" : "OFF");
    printf("- LO Frequency: %.2f MHz\n", freq_mhz);
    printf("- Gain: %d dB\n", tx_gain);
    
    printf("- Antennas enabled: ");
    if (e_tx == 0) {
        printf("None\n");
    } else {
        bool first = true;
        for (int i = 0; i < 4; i++) {
            if (e_tx & (1 << i)) {
                if (!first) printf(", ");
                printf("%d", i + 1);
                first = false;
            }
        }
        printf("\n");
    }

    uint16_t reg_26=0, reg_2f=0, reg_2c=0, reg_2d=0, k_reg=0;
    jtag_read_u16(fd, 0x26, &reg_26);
    jtag_read_u16(fd, 0x2F, &reg_2f);
    jtag_read_u16(fd, 0x2C, &reg_2c);
    jtag_read_u16(fd, 0x2D, &reg_2d);
    jtag_read_u16(fd, 0x27, &k_reg);

    double fs = (k_reg > 0) ? (352.0 / k_reg) : 88.0;
    double tone_freq = (double)(int16_t)reg_2f * fs / 65536.0;
    double p1 = ((reg_2c >> 8) & 0xFF) * 360.0 / 256.0;
    double p2 = (reg_2c & 0xFF) * 360.0 / 256.0;
    double p3 = ((reg_2d >> 8) & 0xFF) * 360.0 / 256.0;
    double p4 = (reg_2d & 0xFF) * 360.0 / 256.0;

    printf("- Test Tone: %s\n", (reg_26 & 0x01) ? "ON" : "OFF");
    printf("- Tone Freq: %.3f MHz\n", tone_freq);
    printf("- Phases: %.1f, %.1f, %.1f, %.1f\n", p1, p2, p3, p4);
    return 0;
}

int max2851_status(int fd)
{
    uint16_t orig_reg14 = max2851_base_regs[14]; // Contains DOUT_SEL = 0
    
    // 1. Check PLL Lock first (ensure DOUT is set to lock detect)
    uint16_t pll_lock_val = 0;
    if (max2851_word(fd, 14, orig_reg14) != 0) return -1;
    max2851_readback(fd, &pll_lock_val);
    bool pll_locked = (pll_lock_val == 0xFFFFu);

    // 2. Set D1 (bit 1) of Main14 to route SPI readback to DOUT
    if (max2851_word(fd, 14, orig_reg14 | (1 << 1)) != 0) return -1;

    uint16_t regs[32] = {0};
    for (int i = 0; i < 32; i++) {
        if (i == 12) continue; // skipped in reg-map
        
        uint16_t cmd = 0x8000 | ((uint16_t)(i & 0x1F) << 10);
        jtag_write_u16(fd, MAX2851_REG_ADDR, cmd);
        jtag_read_u16(fd, MAX2851_REG_ADDR, &regs[i]);
    }

    // 3. Restore Main14 back to default
    max2851_word(fd, 14, orig_reg14);

    // 4. Read additional FPGA-level states
    uint16_t digital_bw_k = 0;
    jtag_read_u16(fd, 0x27, &digital_bw_k);

    uint16_t agc_en = 0;
    uint16_t agc_setpoint = 0;
    jtag_read_u16(fd, 0x6A, &agc_en);
    jtag_read_u16(fd, 0x6B, &agc_setpoint);

    uint16_t pol_val = 0;
    jtag_read_u16(fd, 0x24, &pol_val);

    uint16_t int_val = 0;
    jtag_read_u16(fd, 0x25, &int_val);

    uint16_t r0 = regs[0] & 0x3FF;
    uint16_t r1 = regs[1] & 0x3FF; // LNA + VGA gain bits
    uint16_t r15 = regs[15] & 0x3FF;
    uint16_t r16 = regs[16] & 0x3FF;
    uint16_t r17 = regs[17] & 0x3FF;

    // LO Frequency Calculation
    uint16_t idiv = r15 & 0x7F;
    uint32_t fdiv = ((r16 & 0x3FF) << 10) | (r17 & 0x3FF);
    double ratio = (double)idiv + ((double)fdiv / (double)(1<<20));
    double freq_mhz = ratio * 80.0;

    int bw = (r0 & (1 << 1)) ? 40 : 20;

    uint8_t lna_val = (r1 >> 5) & 0x7;
    uint8_t vga_val = r1 & 0x1F;

    int lna_db = 0;
    switch(lna_val) {
        case 0b000: lna_db = 0;  break;
        case 0b001: lna_db = 8;  break;
        case 0b100: lna_db = 16; break;
        case 0b101: lna_db = 24; break;
        case 0b110: lna_db = 32; break;
        case 0b111: lna_db = 40; break;
        default: lna_db = 0; 
    }

    if (vga_val > 15) {
        vga_val = 15;
    }
    int vga_db = vga_val * 2;
    int total_gain_db = lna_db + vga_db;

    printf("Rx:\n");
    printf("- PLL Lock: %s\n", pll_locked ? "LOCKED" : "UNLOCKED");
    printf("- LO Frequency: %.2f MHz\n", freq_mhz);
    printf("- Gain: %d dB\n", total_gain_db);
    printf("- Analog Bandwidth: %d MHz\n", bw);
    
    if (digital_bw_k > 0) {
        printf("- Digital Bandwidth (k): %u (actual %.2f MHz)\n", digital_bw_k, 240.0 / digital_bw_k);
    } else {
        printf("- Digital Bandwidth (k): 0 (Disabled/Invalid)\n");
    }
    
    printf("- AGC: %s (Setpoint: %u)\n", (agc_en & 0x0080) ? "Enabled" : "Disabled", agc_setpoint);
    printf("- Polarization: %s\n", (pol_val == 0x01) ? "RHCP" : "LHCP");
    printf("- Interleaved Mode: %s\n", (int_val == 0x01) ? "ON" : "OFF");

    uint16_t reg_2e=0, reg_2f=0, reg_2a=0, reg_2b=0;
    jtag_read_u16(fd, 0x2E, &reg_2e);
    jtag_read_u16(fd, 0x2F, &reg_2f);
    jtag_read_u16(fd, 0x2A, &reg_2a);
    jtag_read_u16(fd, 0x2B, &reg_2b);

    /* digital_bw_k is already read earlier in max2851_status */
    double fs = (digital_bw_k > 0) ? (352.0 / digital_bw_k) : 88.0;
    double tone_freq = (double)(int16_t)reg_2f * fs / 65536.0;
    double p1 = ((reg_2a >> 8) & 0xFF) * 360.0 / 256.0;
    double p2 = (reg_2a & 0xFF) * 360.0 / 256.0;
    double p3 = ((reg_2b >> 8) & 0xFF) * 360.0 / 256.0;
    double p4 = (reg_2b & 0xFF) * 360.0 / 256.0;

    printf("- Auto Steer: %s\n", (reg_2e & 0x01) ? "ON" : "OFF");
    printf("- Test Tone: %s\n", (reg_2e & 0x02) ? "ON" : "OFF");
    printf("- Tone Freq: %.3f MHz\n", tone_freq);
    printf("- Phases: %.1f, %.1f, %.1f, %.1f\n", p1, p2, p3, p4);
    return 0;
}