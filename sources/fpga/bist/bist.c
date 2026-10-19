// Build:
//   gcc bist.c -O3 -o bist -lm -lpthread
//
// Run:
//   ./bist
//   ./bist --carrier-mhz 5800 --tone-hz 1000000 --tx-gain 20 --rx-gain 30 --samples 262144
//   ./bist -tx 1            # also estimate and apply TX QEC
//   ./bist --clear-qec      # clear all FPGA QEC coefficients and exit

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "/usr/src/quadrf-fpga-1.0.42/csi/fpga_csi.h"

#ifndef DSI_IOC_MAGIC
#define DSI_IOC_MAGIC 'D'
#endif

struct dsi_fb_info {
    uint64_t fb_bytes;
    uint32_t fb_count;
    uint32_t head;
    uint32_t tail;
    int32_t  queued;
    uint32_t _pad;
};

#ifndef DSI_IOC_GET_FB_INFO
#define DSI_IOC_GET_FB_INFO _IOR(DSI_IOC_MAGIC, 0x10, struct dsi_fb_info)
#endif
#ifndef DSI_IOC_QUEUE_NEXT
#define DSI_IOC_QUEUE_NEXT _IO(DSI_IOC_MAGIC, 0x11)
#endif

enum { CHANNELS = 4, BYTES_PER_IQ4 = 8 };

#define RX_DEVICE "/dev/csi_stream0"
#define TX_DEVICE "/dev/dsi_stream0"

#define MAX2850_REG_ADDR 0x42
#define MAX2851_REG_ADDR 0x43

#define TX_QEC_REG       0x26
#define RX_QEC_COEFF_REG 0x68
#define RX_QEC_INDEX_REG 0x69

#define DEFAULT_RF_FREQ_MHZ  5800.0
#define DEFAULT_TONE_FREQ_HZ 1000000.0
#define DEFAULT_RX_GAIN_WORD 30u
#define DEFAULT_TX_GAIN_WORD 20u

// Exact line-rate sample rates used by the working QuadRF design.
#define EXACT_TX_RATE \
    (0.5 * 175000000.0 * (3317760.0 / 3372462.0))
#define EXACT_RX_RATE \
    (((65536.0 / 76714.0) * 175000000.0) / 4.0)
#define TX_PAYLOAD_BYTES 3317760u

// Samples are IQ4 frames: one complex CS8 sample from every RX channel.
#define SETTLE_SAMPLES              8192u
#define DEFAULT_INTEGRATION_SAMPLES (65536u * 4u)
#define TONE_SWITCH_SETTLE_SECONDS  0.100

struct config {
    double rf_freq_mhz;
    double tone_freq_hz;
    unsigned rx_gain_word;
    unsigned tx_gain_word;
    uint32_t integration_samples;
    bool tx_qec_enabled;
    bool clear_qec_only;
};

static struct config g_config = {
    .rf_freq_mhz = DEFAULT_RF_FREQ_MHZ,
    .tone_freq_hz = DEFAULT_TONE_FREQ_HZ,
    .rx_gain_word = DEFAULT_RX_GAIN_WORD,
    .tx_gain_word = DEFAULT_TX_GAIN_WORD,
    .integration_samples = DEFAULT_INTEGRATION_SAMPLES,
    .tx_qec_enabled = false,
    .clear_qec_only = false,
};

// Correlation matrix between captured RX [I,Q] rows and transmitted
// baseband [I,Q] columns.
struct correlation_matrix {
    double ii;
    double iq;
    double qi;
    double qq;
};

struct rx_covariance {
    double ii;
    double iq;
    double qq;
};

struct measurement {
    struct correlation_matrix tx;
    struct rx_covariance rx;
    double reference_energy;
};

struct measurement_accumulator {
    double sum_i;
    double sum_q;
    double sum_ref_i;
    double sum_ref_q;
    double reference_energy;
    double rx_ii;
    double rx_iq;
    double rx_qq;
    struct correlation_matrix tx;
};

struct qec_coefficients {
    double rx_gain;
    double rx_phase;
    double tx_gain;
    double tx_phase;
    int8_t rx_gain_code;
    int8_t rx_phase_code;
    int8_t tx_gain_code;
    int8_t tx_phase_code;
    bool rx_valid;
    bool tx_valid;
};

// Human-readable imbalance estimates in the convention used by the original
// BIST and Android calibration code.  These remain useful for comparing the
// before/after measurements even though the FPGA datapaths use different
// triangular correction matrices and fixed-point coefficient scales.
struct quadrature_measurement {
    double rx_gain;
    double rx_phase;
    double tx_gain;
    double tx_phase;
    bool rx_valid;
    bool tx_valid;
};

enum {
    OPT_TX_QEC = 1000,
    OPT_CLEAR_QEC
};

#define LUT_BITS 16u
#define LUT_SIZE (1u << LUT_BITS)

static float  g_cos_lut[LUT_SIZE];
static float  g_sin_lut[LUT_SIZE];
static int8_t g_cos_i8[LUT_SIZE];
static int8_t g_sin_i8[LUT_SIZE];

static volatile sig_atomic_t g_stop;
static atomic_bool g_radio_ready = ATOMIC_VAR_INIT(false);
static atomic_bool g_tx_streaming = ATOMIC_VAR_INIT(false);
static atomic_bool g_tx_failed = ATOMIC_VAR_INIT(false);
static atomic_int g_requested_polarity = ATOMIC_VAR_INIT(1);
static atomic_uint_fast64_t g_requested_step = ATOMIC_VAR_INIT(0);
static atomic_uint g_requested_generation = ATOMIC_VAR_INIT(0);
static atomic_uint g_queued_generation = ATOMIC_VAR_INIT(0);

static void on_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

static void print_usage(FILE *stream, const char *program)
{
    fprintf(stream,
        "Usage: %s [OPTIONS]\n"
        "\n"
        "Measure the full QuadRF 4x4 BIST loopback channel and direct-path QEC.\n"
        "\n"
        "Options:\n"
        "  -c, --carrier-mhz MHZ  RF carrier in MHz (default: %.0f)\n"
        "  -t, --tone-hz HZ       Complex tone frequency in Hz (default: %.0f)\n"
        "  -x, --tx-gain WORD     TX gain word, 0..63 (default: %u)\n"
        "  -r, --rx-gain WORD     RX gain word, 0..63 (default: %u)\n"
        "  -n, --samples COUNT    Integration samples per TX path (default: %u)\n"
        "      -tx 0|1            Calculate and apply TX QEC (default: 0)\n"
        "      --clear-qec        Clear all FPGA QEC coefficients and exit\n"
        "  -h, --help             Show this help and exit\n",
        program,
        DEFAULT_RF_FREQ_MHZ,
        DEFAULT_TONE_FREQ_HZ,
        DEFAULT_TX_GAIN_WORD,
        DEFAULT_RX_GAIN_WORD,
        DEFAULT_INTEGRATION_SAMPLES);
}

static int parse_double_arg(const char *text, double *value)
{
    char *end;

    errno = 0;
    double parsed = strtod(text, &end);
    if (errno == ERANGE || end == text || *end != '\0' || !isfinite(parsed))
        return -1;

    *value = parsed;
    return 0;
}

static int parse_unsigned_arg(const char *text, uint64_t maximum,
                              uint64_t *value)
{
    char *end;

    if (text[0] == '-')
        return -1;

    errno = 0;
    unsigned long long parsed = strtoull(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0' || parsed > maximum)
        return -1;

    *value = (uint64_t)parsed;
    return 0;
}

static int parse_options(int argc, char **argv)
{
    static const struct option options[] = {
        { "carrier-mhz", required_argument, NULL, 'c' },
        { "carrier",     required_argument, NULL, 'c' },
        { "tone-hz",     required_argument, NULL, 't' },
        { "tone",        required_argument, NULL, 't' },
        { "tx-gain",     required_argument, NULL, 'x' },
        { "rx-gain",     required_argument, NULL, 'r' },
        { "samples",     required_argument, NULL, 'n' },
        { "sample-size", required_argument, NULL, 'n' },
        { "tx",          required_argument, NULL, OPT_TX_QEC },
        { "tx-qec",      required_argument, NULL, OPT_TX_QEC },
        { "clear-qec",    no_argument,      NULL, OPT_CLEAR_QEC },
        { "help",        no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    for (;;) {
        // getopt_long_only accepts the requested multi-character -tx form.
        // The existing one-letter options, including -t for tone, remain
        // unambiguous because "tx" is an exact long-option match.
        int option = getopt_long_only(
            argc, argv, "c:t:x:r:n:h", options, NULL);
        if (option == -1)
            break;

        uint64_t parsed_unsigned;
        switch (option) {
        case 'c':
            if (parse_double_arg(optarg, &g_config.rf_freq_mhz) < 0 ||
                g_config.rf_freq_mhz < 4900.0 ||
                g_config.rf_freq_mhz > 6000.0) {
                fprintf(stderr,
                        "Invalid carrier frequency '%s' (expected 4900..6000 MHz)\n",
                        optarg);
                return -1;
            }
            break;
        case 't':
            if (parse_double_arg(optarg, &g_config.tone_freq_hz) < 0 ||
                g_config.tone_freq_hz <= 0.0 ||
                g_config.tone_freq_hz >= EXACT_RX_RATE / 2.0) {
                fprintf(stderr,
                        "Invalid tone frequency '%s' (expected 0 < HZ < %.0f)\n",
                        optarg, EXACT_RX_RATE / 2.0);
                return -1;
            }
            break;
        case 'x':
            if (parse_unsigned_arg(optarg, 63, &parsed_unsigned) < 0) {
                fprintf(stderr,
                        "Invalid TX gain '%s' (expected integer 0..63)\n",
                        optarg);
                return -1;
            }
            g_config.tx_gain_word = (unsigned)parsed_unsigned;
            break;
        case 'r':
            if (parse_unsigned_arg(optarg, 63, &parsed_unsigned) < 0) {
                fprintf(stderr,
                        "Invalid RX gain '%s' (expected integer 0..63)\n",
                        optarg);
                return -1;
            }
            g_config.rx_gain_word = (unsigned)parsed_unsigned;
            break;
        case 'n':
            if (parse_unsigned_arg(optarg, UINT32_MAX, &parsed_unsigned) < 0 ||
                parsed_unsigned == 0) {
                fprintf(stderr,
                        "Invalid sample size '%s' (expected integer 1..%u)\n",
                        optarg, UINT32_MAX);
                return -1;
            }
            g_config.integration_samples = (uint32_t)parsed_unsigned;
            break;
        case OPT_TX_QEC:
            if (parse_unsigned_arg(optarg, 1, &parsed_unsigned) < 0) {
                fprintf(stderr,
                        "Invalid TX QEC setting '%s' (expected 0 or 1)\n",
                        optarg);
                return -1;
            }
            g_config.tx_qec_enabled = parsed_unsigned != 0;
            break;
        case OPT_CLEAR_QEC:
            g_config.clear_qec_only = true;
            break;
        case 'h':
            print_usage(stdout, argv[0]);
            return 1;
        default:
            print_usage(stderr, argv[0]);
            return -1;
        }
    }

    if (optind != argc) {
        fprintf(stderr, "Unexpected positional argument: %s\n", argv[optind]);
        print_usage(stderr, argv[0]);
        return -1;
    }

    return 0;
}

static void init_luts(void)
{
    for (uint32_t n = 0; n < LUT_SIZE; n++) {
        double a = 2.0 * M_PI * (double)n / (double)LUT_SIZE;
        g_cos_lut[n] = (float)cos(a);
        g_sin_lut[n] = (float)sin(a);
        g_cos_i8[n] = (int8_t)lround(127.0 * cos(a));
        g_sin_i8[n] = (int8_t)lround(127.0 * sin(a));
    }
}

static uint64_t dds_step(double frequency_hz, double sample_rate_hz)
{
    return (uint64_t)((frequency_hz / sample_rate_hz) *
                      18446744073709551616.0); // 2^64
}

static void fill_tone(int8_t *dst, size_t samples,
                      uint64_t *phase, uint64_t step, int polarity)
{
    uint64_t p = *phase;

    for (size_t n = 0; n < samples; n++) {
        uint32_t k = (uint32_t)(p >> (64u - LUT_BITS));
        dst[2 * n + 0] = g_cos_i8[k];
        dst[2 * n + 1] = polarity > 0
                       ? g_sin_i8[k] : (int8_t)-g_sin_i8[k];
        p += step;
    }
    *phase = p;
}

static int jtag_write_u16(int fd, uint8_t addr, uint16_t value)
{
    struct csi_jtag_reg r = { .addr = addr, .value = value };

    for (int attempt = 0; attempt < 100; attempt++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) == 0)
            return 0;
        if (errno == EINTR)
            continue;
        if (errno != EBUSY)
            break;
        usleep(1000);
    }
    return -1;
}

static int jtag_read_u16(int fd, uint8_t addr, uint16_t *value)
{
    struct csi_jtag_reg r = { .addr = addr, .value = 0 };

    for (int attempt = 0; attempt < 100; attempt++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) == 0) {
            *value = r.value;
            return 0;
        }
        if (errno == EINTR)
            continue;
        if (errno != EBUSY)
            break;
        usleep(1000);
    }
    return -1;
}

static int jtag_acquire_lease(int fd)
{
    while (!g_stop) {
        if (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) == 0 || errno == ENOTTY)
            return 0;
        if (errno != EBUSY)
            return -1;
        usleep(1000);
    }
    errno = EINTR;
    return -1;
}

static uint16_t pack_rx_qec_word(int8_t gain, int8_t phase)
{
    return (uint16_t)(((uint16_t)(uint8_t)gain << 8) |
                      (uint16_t)(uint8_t)phase);
}

static int program_rx_qec(int fd,
                          const struct qec_coefficients qec[CHANNELS],
                          bool clear)
{
    // Deassert every load strobe before changing the shared coefficient bus.
    if (jtag_write_u16(fd, RX_QEC_INDEX_REG, 0) < 0)
        return -1;

    if (clear) {
        if (jtag_write_u16(fd, RX_QEC_COEFF_REG, 0) < 0)
            return -1;
        for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
            if (jtag_write_u16(fd, RX_QEC_INDEX_REG,
                               (uint16_t)(antenna + 1u)) < 0)
                return -1;
        }
    } else {
        for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
            uint16_t word = pack_rx_qec_word(
                qec[antenna].rx_gain_code, qec[antenna].rx_phase_code);

            if (jtag_write_u16(fd, RX_QEC_INDEX_REG, 0) < 0 ||
                jtag_write_u16(fd, RX_QEC_COEFF_REG, word) < 0 ||
                jtag_write_u16(fd, RX_QEC_INDEX_REG,
                               (uint16_t)(antenna + 1u)) < 0)
                return -1;
        }
    }

    return jtag_write_u16(fd, RX_QEC_INDEX_REG, 0);
}

static int program_tx_qec(int fd,
                          const struct qec_coefficients qec[CHANNELS],
                          bool clear)
{
    // The coefficients below have no datapath effect if the FPGA project was
    // built with BYPASS_TX_QEC defined.  tx_path still retains and loads the
    // five coefficient registers in that build, so successful JTAG writes do
    // not by themselves prove that TX QEC is enabled in the bitstream.
    uint16_t readback;
    if (jtag_read_u16(fd, TX_QEC_REG, &readback) < 0)
        return -1;

    // Writes to 0x26 also carry the tone/LFSR/LMS control bits.
    uint16_t control = (uint16_t)(readback & 0x0007u);

    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        uint8_t gain = clear ? 0u : (uint8_t)qec[antenna].tx_gain_code;
        uint8_t phase = clear ? 0u : (uint8_t)qec[antenna].tx_phase_code;

        // tx_path is a five-byte shift chain.  Load in reverse register order:
        // DC_Q, DC_I, delay, phase, gain.
        uint8_t sequence[5] = { 0u, 0u, 0u, phase, gain };
        uint16_t index = (uint16_t)((antenna + 1u) << 4);

        for (unsigned n = 0; n < 5; n++) {
            uint16_t word = (uint16_t)(((uint16_t)sequence[n] << 8) |
                                       index | control);
            if (jtag_write_u16(fd, TX_QEC_REG, word) < 0)
                return -1;
        }

        // Explicitly remove the load selection after the final shift.
        if (jtag_write_u16(fd, TX_QEC_REG, control) < 0)
            return -1;
    }
    return 0;
}

static int program_qec(int fd,
                       const struct qec_coefficients qec[CHANNELS],
                       bool clear, bool include_tx)
{
    if (program_rx_qec(fd, qec, clear) < 0)
        return -1;
    if (include_tx && program_tx_qec(fd, qec, clear) < 0)
        return -1;

    // JTAG transactions are already slow relative to both datapath clocks;
    // this is only a final guard before capture or shutdown.
    usleep(1000);
    return 0;
}

static int spi_word(int fd, uint8_t fpga_addr,
                    uint16_t main_reg, uint16_t data10)
{
    uint16_t word = (uint16_t)(((main_reg & 0x3fu) << 10) |
                               (data10 & 0x03ffu));
    return jtag_write_u16(fd, fpga_addr, word);
}

static int set_lo_mhz(int fd, uint8_t spi_addr, double frequency_mhz)
{
    double ratio = frequency_mhz / 80.0;
    long long integer = (long long)floor(ratio);
    long long fraction = llround((ratio - (double)integer) * (1u << 20));

    if (fraction == (1LL << 20)) {
        integer++;
        fraction = 0;
    }

    if (jtag_write_u16(fd, spi_addr,
            (uint16_t)((15u << 10) | (1u << 9) |
                       ((unsigned)integer & 0x7fu))) < 0 ||
        jtag_write_u16(fd, spi_addr,
            (uint16_t)((16u << 10) |
                       (((unsigned)fraction >> 10) & 0x03ffu))) < 0 ||
        jtag_write_u16(fd, spi_addr,
            (uint16_t)((17u << 10) |
                       ((unsigned)fraction & 0x03ffu))) < 0)
        return -1;

    // Select the MAX2851 LNA band for the requested carrier.  The highest
    // band is also used above 5.9 GHz for the QuadRF's extended range.
    if (spi_addr == MAX2851_REG_ADDR) {
        unsigned lna_band = frequency_mhz < 5200.0 ? 0u
                          : frequency_mhz < 5500.0 ? 1u
                          : frequency_mhz < 5800.0 ? 2u : 3u;
        return spi_word(fd, spi_addr, 2,
                        (uint16_t)(0x0180u | (lna_band << 5)));
    }
    return 0;
}

static int retune_tx_lo_mhz(int fd, double frequency_mhz)
{
    if (set_lo_mhz(fd, MAX2850_REG_ADDR, frequency_mhz) < 0 ||
        spi_word(fd, MAX2850_REG_ADDR, 14, 0x0160) < 0)
        return -1;
    usleep(20000);
    return 0;
}

static int select_tx(int fd, unsigned tx)
{
    uint16_t mask = (uint16_t)(1u << tx);
    uint16_t max2850_reg0 = (uint16_t)(0x000eu | (mask << 5));

    if (spi_word(fd, MAX2850_REG_ADDR, 0, max2850_reg0) < 0)
        return -1;
    return jtag_write_u16(fd, 0x02, mask);
}

static int setup_radio(int fd, const struct config *config)
{
    uint16_t value;

    if (ioctl(fd, CSI_IOC_JTAG_SETUP) < 0 && errno != ENOTTY)
        perror("CSI_IOC_JTAG_SETUP");

    // Four-channel RX interleave; no upstream-MIPI summing; 40 MHz filter.
    if (jtag_write_u16(fd, 0x25, 1) < 0)
        return -1;
    if (jtag_read_u16(fd, 0x2e, &value) < 0 ||
        jtag_write_u16(fd, 0x2e, (uint16_t)(value & ~0x0002u)) < 0 ||
        jtag_write_u16(fd, 0x27, 6) < 0)
        return -1;

    // FPGA test tone off: DSI supplies the transmitted IQ samples.
    if (jtag_read_u16(fd, TX_QEC_REG, &value) < 0 ||
        jtag_write_u16(fd, TX_QEC_REG,
                       (uint16_t)(value & ~0x0001u)) < 0)
        return -1;

    // MAX2851: all four RX paths, RX mode, 40 MHz analog bandwidth.
    if (spi_word(fd, MAX2851_REG_ADDR, 6, 0x03ff) < 0 ||
        spi_word(fd, MAX2851_REG_ADDR, 0, 0x000a) < 0 ||
        set_lo_mhz(fd, MAX2851_REG_ADDR, config->rf_freq_mhz) < 0 ||
        jtag_write_u16(fd, 0x6a, (uint16_t)config->rx_gain_word) < 0)
        return -1;

    // Analog TX path, PA bias, TX LO locked to RX, and TX gain.
    if (jtag_write_u16(fd, 0x23, 0) < 0 ||
        spi_word(fd, MAX2850_REG_ADDR, 10, 1) < 0 ||
        jtag_write_u16(fd, 0x6d, 1) < 0 ||
        set_lo_mhz(fd, MAX2850_REG_ADDR, config->rf_freq_mhz) < 0 ||
        spi_word(fd, MAX2850_REG_ADDR, 9,
                 (uint16_t)((config->tx_gain_word << 4) | 0x0f)) < 0)
        return -1;

    // RHCP, then restrobe four-channel interleave after all SPI traffic.
    if (jtag_write_u16(fd, 0x24, 1) < 0 ||
        jtag_write_u16(fd, 0x25, 1) < 0 ||
        select_tx(fd, 0) < 0)
        return -1;

    // Run VAS and allow both PLLs to settle.
    if (spi_word(fd, MAX2851_REG_ADDR, 14, 0x0160) < 0 ||
        spi_word(fd, MAX2850_REG_ADDR, 14, 0x0160) < 0)
        return -1;
    usleep(20000);
    return 0;
}

static void radio_standby(int fd)
{
    uint16_t reg0 = (uint16_t)((0x000eu & ~(0x7u << 2)) | (1u << 1));

    if (spi_word(fd, MAX2850_REG_ADDR, 0, reg0) < 0)
        perror("MAX2850 standby");
    if (jtag_write_u16(fd, 0x23, 1) < 0)
        perror("FPGA TX disable");
    if (spi_word(fd, MAX2850_REG_ADDR, 10, 0) < 0)
        perror("MAX2850 PA bias off");
    if (jtag_write_u16(fd, 0x02, 0) < 0)
        perror("FPGA TX mask clear");
}

static void *tx_worker(void *unused)
{
    (void)unused;

    while (!g_stop && !atomic_load(&g_radio_ready))
        usleep(1000);
    if (g_stop)
        return NULL;

    int fd = open(TX_DEVICE, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("open " TX_DEVICE);
        atomic_store(&g_tx_failed, true);
        return NULL;
    }

    struct dsi_fb_info info = {0};
    for (int n = 0; n < 200 && !g_stop; n++) {
        if (ioctl(fd, DSI_IOC_GET_FB_INFO, &info) == 0 &&
            info.fb_bytes && info.fb_count)
            break;
        usleep(5000);
    }
    if (!info.fb_bytes || !info.fb_count) {
        info.fb_bytes = TX_PAYLOAD_BYTES;
        info.fb_count = 3;
    }

    size_t frame_bytes = (size_t)info.fb_bytes;
    size_t frame_samples = frame_bytes / 2;
    size_t map_len = (size_t)info.fb_count * frame_bytes;
    size_t page = (size_t)sysconf(_SC_PAGESIZE);
    map_len = (map_len + page - 1) & ~(page - 1);

    int8_t *mapped = mmap(NULL, map_len, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, 0);
    bool use_mmap = mapped != MAP_FAILED;
    int8_t *fallback = NULL;

    if (!use_mmap) {
        fallback = malloc(frame_bytes);
        if (!fallback) {
            perror("allocate TX buffer");
            atomic_store(&g_tx_failed, true);
            close(fd);
            return NULL;
        }
    }

    struct pollfd pfd = { .fd = fd, .events = POLLOUT };
    uint64_t phase = 0;

    while (!g_stop) {
        int pr = poll(&pfd, 1, 20);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (pr == 0)
            continue;

        int8_t *dst = fallback;
        if (use_mmap) {
            if (ioctl(fd, DSI_IOC_GET_FB_INFO, &info) < 0)
                continue;
            dst = mapped + (size_t)(info.head % info.fb_count) * frame_bytes;
        }

        unsigned generation = atomic_load_explicit(
            &g_requested_generation, memory_order_acquire);
        uint64_t step = atomic_load_explicit(
            &g_requested_step, memory_order_relaxed);
        int polarity = atomic_load_explicit(
            &g_requested_polarity, memory_order_relaxed);
        uint64_t next_phase = phase;
        fill_tone(dst, frame_samples, &next_phase, step, polarity);

        bool queued = false;
        if (use_mmap) {
            if (ioctl(fd, DSI_IOC_QUEUE_NEXT) == 0)
                queued = true;
            else if (errno != EAGAIN)
                break;
        } else {
            ssize_t written = write(fd, fallback, frame_bytes);
            if (written == (ssize_t)frame_bytes)
                queued = true;
            else if (written < 0 && errno != EAGAIN && errno != EINTR)
                break;
        }

        if (queued) {
            phase = next_phase;
            atomic_store_explicit(&g_queued_generation, generation,
                                  memory_order_release);
            atomic_store(&g_tx_streaming, true);
        }
    }

    if (!g_stop)
        atomic_store(&g_tx_failed, true);
    if (use_mmap)
        munmap(mapped, map_len);
    else
        free(fallback);
    close(fd);
    return NULL;
}

static uint32_t ring_used(const struct csi_ring_info *ri)
{
    return ri->head >= ri->tail
         ? ri->head - ri->tail
         : ri->ring_size - (ri->tail - ri->head);
}

// Discard everything already delivered, while advancing the RX DDS by the
// same number of IQ4 sample times so its phase remains continuous.
static int discard_backlog(int fd, uint64_t *phase, uint64_t step)
{
    struct csi_ring_info ri;

    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
        return -1;

    uint32_t bytes = ring_used(&ri);
    bytes -= bytes % BYTES_PER_IQ4;
    if (bytes == 0)
        return 0;

    uint32_t samples = bytes / BYTES_PER_IQ4;
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &bytes) < 0)
        return -1;
    *phase += step * (uint64_t)samples;
    return 0;
}

static void finalize_measurement(const struct measurement_accumulator *acc,
                                 uint32_t count,
                                 struct measurement *result)
{
    double inv_count = 1.0 / (double)count;

    // Center both captured and reference samples.  This removes DC leakage
    // and the small mean left when the integration contains a noninteger
    // number of tone periods.
    result->rx.ii = acc->rx_ii - acc->sum_i * acc->sum_i * inv_count;
    result->rx.iq = acc->rx_iq - acc->sum_i * acc->sum_q * inv_count;
    result->rx.qq = acc->rx_qq - acc->sum_q * acc->sum_q * inv_count;

    result->tx.ii = acc->tx.ii -
                    acc->sum_i * acc->sum_ref_i * inv_count;
    result->tx.iq = acc->tx.iq -
                    acc->sum_i * acc->sum_ref_q * inv_count;
    result->tx.qi = acc->tx.qi -
                    acc->sum_q * acc->sum_ref_i * inv_count;
    result->tx.qq = acc->tx.qq -
                    acc->sum_q * acc->sum_ref_q * inv_count;
    result->reference_energy = acc->reference_energy -
        (acc->sum_ref_i * acc->sum_ref_i +
         acc->sum_ref_q * acc->sum_ref_q) * inv_count;
}

// Consume exactly count IQ4 sample times.  With result == NULL, samples are
// discarded.  Otherwise calculate RX covariance and RX-vs-TX correlations
// for all four channels.  The DDS phase always advances in the positive
// direction; a negative-frequency tone is represented by negating ref_q.
static int consume_samples(int fd, const int8_t *ring, uint32_t ring_size,
                           uint32_t count, uint64_t *phase, uint64_t step,
                           int polarity,
                           struct measurement result[CHANNELS])
{
    uint32_t done = 0;
    int idle_ms = 0;
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    struct measurement_accumulator acc[CHANNELS] = {{0}};

    while (done < count && !g_stop) {
        struct csi_ring_info ri;
        if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
            return -1;

        uint32_t used = ring_used(&ri);
        uint32_t available = used / BYTES_PER_IQ4;
        if (available == 0) {
            int pr = poll(&pfd, 1, 100);
            if (pr < 0 && errno != EINTR)
                return -1;
            if (pr == 0) {
                idle_ms += 100;
                if (idle_ms >= 2000 || atomic_load(&g_tx_failed)) {
                    errno = ETIMEDOUT;
                    return -1;
                }
            }
            continue;
        }
        idle_ms = 0;

        uint32_t contiguous = (ring_size - ri.tail) / BYTES_PER_IQ4;
        uint32_t batch = available < contiguous ? available : contiguous;
        if (batch > count - done)
            batch = count - done;

        const int8_t *src = ring + ri.tail;
        if (result) {
            for (uint32_t n = 0; n < batch; n++) {
                uint32_t k = (uint32_t)(*phase >> (64u - LUT_BITS));
                double ref_i = g_cos_lut[k];
                double ref_q = polarity > 0
                             ? g_sin_lut[k] : -g_sin_lut[k];

                for (unsigned rx = 0; rx < CHANNELS; rx++) {
                    int8_t raw_i = src[n * BYTES_PER_IQ4 + 2 * rx + 0];
                    int8_t raw_q = src[n * BYTES_PER_IQ4 + 2 * rx + 1];
                    double i = (double)raw_i / 127.0;
                    double q = (double)raw_q / 127.0;
                    struct measurement_accumulator *a = &acc[rx];

                    a->sum_i += i;
                    a->sum_q += q;
                    a->sum_ref_i += ref_i;
                    a->sum_ref_q += ref_q;
                    a->reference_energy += ref_i * ref_i + ref_q * ref_q;

                    a->rx_ii += i * i;
                    a->rx_iq += i * q;
                    a->rx_qq += q * q;

                    a->tx.ii += i * ref_i;
                    a->tx.iq += i * ref_q;
                    a->tx.qi += q * ref_i;
                    a->tx.qq += q * ref_q;
                }
                *phase += step;
            }
        } else {
            *phase += step * (uint64_t)batch;
        }

        uint32_t bytes = batch * BYTES_PER_IQ4;
        if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &bytes) < 0)
            return -1;
        done += batch;
    }

    if (g_stop) {
        errno = EINTR;
        return -1;
    }

    if (result) {
        for (unsigned rx = 0; rx < CHANNELS; rx++)
            finalize_measurement(&acc[rx], count, &result[rx]);
    }
    return 0;
}

static int set_tx_tone(int fd, const int8_t *ring, uint32_t ring_size,
                       uint64_t *rx_phase, uint64_t rx_step,
                       double signed_frequency_hz)
{
    double frequency_hz = fabs(signed_frequency_hz);
    if (!(frequency_hz > 0.0) || frequency_hz >= EXACT_TX_RATE / 2.0 ||
        !isfinite(frequency_hz)) {
        errno = EINVAL;
        return -1;
    }

    int polarity = signed_frequency_hz > 0.0 ? 1 : -1;
    uint64_t tx_step = dds_step(frequency_hz, EXACT_TX_RATE);
    unsigned generation = atomic_load_explicit(
        &g_requested_generation, memory_order_relaxed) + 1u;

    atomic_store_explicit(&g_requested_step, tx_step, memory_order_relaxed);
    atomic_store_explicit(&g_requested_polarity, polarity,
                          memory_order_relaxed);
    atomic_store_explicit(&g_requested_generation, generation,
                          memory_order_release);

    // Wait until the TX worker has queued a buffer with the new tone.
    // Continue draining RX so its small ring cannot overflow while waiting.
    for (int n = 0; n < 2000 && !g_stop; n++) {
        if (discard_backlog(fd, rx_phase, rx_step) < 0)
            return -1;
        if (atomic_load_explicit(&g_queued_generation,
                                 memory_order_acquire) == generation)
            break;
        if (atomic_load(&g_tx_failed)) {
            errno = EIO;
            return -1;
        }
        usleep(1000);
    }

    if (atomic_load_explicit(&g_queued_generation,
                             memory_order_acquire) != generation) {
        errno = ETIMEDOUT;
        return -1;
    }

    // Old-tone framebuffers may already be ahead of the newly queued buffer.
    // Drain 100 ms, longer than a full triple-buffered DSI pipeline.
    uint32_t settle_samples =
        (uint32_t)llround(EXACT_RX_RATE * TONE_SWITCH_SETTLE_SECONDS);
    return consume_samples(fd, ring, ring_size, settle_samples,
                           rx_phase, rx_step, polarity, NULL);
}

static int capture_all_paths(
    int fd, const int8_t *ring, uint32_t ring_size,
    uint64_t *rx_phase, uint64_t rx_step, int reference_polarity,
    uint32_t integration_samples,
    struct measurement measurements[CHANNELS][CHANNELS])
{
    for (unsigned tx = 0; tx < CHANNELS; tx++) {
        if (select_tx(fd, tx) < 0)
            return -1;

        // Remove samples captured under the preceding TX selection, then
        // wait through the RF/FPGA switching transient.
        if (discard_backlog(fd, rx_phase, rx_step) < 0 ||
            consume_samples(fd, ring, ring_size, SETTLE_SAMPLES,
                            rx_phase, rx_step,
                            reference_polarity, NULL) < 0)
            return -1;

        struct measurement column[CHANNELS];
        if (consume_samples(fd, ring, ring_size, integration_samples,
                            rx_phase, rx_step,
                            reference_polarity, column) < 0)
            return -1;

        for (unsigned rx = 0; rx < CHANNELS; rx++)
            measurements[rx][tx] = column[rx];
    }
    return 0;
}

static int capture_qec_cycle(
    int fd, const int8_t *ring, uint32_t ring_size,
    uint64_t *rx_phase, uint64_t rx_step,
    const struct config *config, const char *stage,
    struct measurement rx_positive[CHANNELS][CHANNELS],
    struct measurement rx_negative[CHANNELS][CHANNELS],
    struct measurement tx_positive[CHANNELS][CHANNELS],
    struct measurement tx_negative[CHANNELS][CHANNELS])
{
    // Offset the TX LO during RX-QEC so TX desired and image tones do not
    // overlap coherently at +/-tone.  This makes their aggregate covariance
    // circular before RX imbalance, allowing the RX correction to be
    // separated from TX imbalance.  A tone-relative offset avoids a DC
    // baseband test at the default 1 MHz tone.
    double offset_magnitude_hz = fmin(1000000.0,
                                      config->tone_freq_hz * 0.25);
    double lo_offset_hz =
        config->rf_freq_mhz + offset_magnitude_hz / 1e6 <= 6000.0
        ? offset_magnitude_hz : -offset_magnitude_hz;
    double rx_qec_tx_lo_mhz = config->rf_freq_mhz + lo_offset_hz / 1e6;
    double rx_qec_positive_bb_hz = config->tone_freq_hz - lo_offset_hz;
    double rx_qec_negative_bb_hz = -config->tone_freq_hz - lo_offset_hz;

    printf("\n%s\n", stage);
    printf("RX-QEC: TX LO offset %+.3f MHz\n", lo_offset_hz / 1e6);
    fflush(stdout);
    if (retune_tx_lo_mhz(fd, rx_qec_tx_lo_mhz) < 0) {
        perror("set RX-QEC TX LO offset");
        return -1;
    }

    printf("Measuring RX-QEC +%.3f MHz direct loopbacks\n",
           config->tone_freq_hz / 1e6);
    fflush(stdout);
    if (set_tx_tone(fd, ring, ring_size, rx_phase, rx_step,
                    rx_qec_positive_bb_hz) < 0 ||
        capture_all_paths(fd, ring, ring_size, rx_phase, rx_step, 1,
                          config->integration_samples, rx_positive) < 0) {
        perror("measure positive RX-QEC tone");
        return -1;
    }

    printf("Measuring RX-QEC -%.3f MHz direct loopbacks\n",
           config->tone_freq_hz / 1e6);
    fflush(stdout);
    if (set_tx_tone(fd, ring, ring_size, rx_phase, rx_step,
                    rx_qec_negative_bb_hz) < 0 ||
        capture_all_paths(fd, ring, ring_size, rx_phase, rx_step, -1,
                          config->integration_samples, rx_negative) < 0) {
        perror("measure negative RX-QEC tone");
        return -1;
    }

    // Equal TX/RX LOs are required for the nominal TX imbalance report and,
    // when enabled, the full TX-QEC A/B/C/D correction calculation.
    if (retune_tx_lo_mhz(fd, config->rf_freq_mhz) < 0) {
        perror("restore TX LO for TX measurement");
        return -1;
    }

    printf("Measuring nominal TX imbalance +%.3f MHz direct loopbacks\n",
           config->tone_freq_hz / 1e6);
    fflush(stdout);
    if (set_tx_tone(fd, ring, ring_size, rx_phase, rx_step,
                    config->tone_freq_hz) < 0 ||
        capture_all_paths(fd, ring, ring_size, rx_phase, rx_step, 1,
                          config->integration_samples, tx_positive) < 0) {
        perror("measure positive TX imbalance tone");
        return -1;
    }

    printf("Measuring nominal TX imbalance -%.3f MHz direct loopbacks\n",
           config->tone_freq_hz / 1e6);
    fflush(stdout);
    if (set_tx_tone(fd, ring, ring_size, rx_phase, rx_step,
                    -config->tone_freq_hz) < 0 ||
        capture_all_paths(fd, ring, ring_size, rx_phase, rx_step, -1,
                          config->integration_samples, tx_negative) < 0) {
        perror("measure negative TX imbalance tone");
        return -1;
    }

    return 0;
}

static bool calculate_rx_metrics(const struct measurement *positive,
                                 const struct measurement *negative,
                                 double *measured_gain,
                                 double *measured_phase,
                                 double *correction_gain,
                                 double *correction_phase)
{
    double x11 = positive->rx.ii + negative->rx.ii;
    double x12 = positive->rx.iq + negative->rx.iq;
    double x22 = positive->rx.qq + negative->rx.qq;
    double determinant = x11 * x22 - x12 * x12;

    if (!(x11 > 0.0) || !(x22 > 0.0) || !(determinant > 0.0) ||
        !isfinite(determinant))
        return false;

    double root_determinant = sqrt(determinant);

    // Original reporting convention: whiten Q using I as the unchanged axis.
    // These values describe the measured imbalance and are retained so runs
    // before and after FPGA correction can be compared directly.
    *measured_phase = -x12 / root_determinant;
    *measured_gain = x11 / root_determinant;

    // Current FPGA convention:
    //   I' = I + phase*Q
    //   Q' = (1 + gain)*Q
    // Choose phase to cancel covariance, then gain to equalize variances.
    *correction_phase = -x12 / x22;
    *correction_gain = root_determinant / x22 - 1.0;

    return isfinite(*measured_gain) && isfinite(*measured_phase) &&
           isfinite(*correction_gain) && isfinite(*correction_phase);
}

// Apply the calculated RX correction to correlation rows without modifying
// captured samples.  This exactly mirrors the FPGA RX-QEC datapath.
static struct correlation_matrix correct_rx_correlations(
    const struct correlation_matrix *input, double gain, double phase)
{
    double q_scale = 1.0 + gain;
    struct correlation_matrix output = {
        .ii = input->ii + phase * input->qi,
        .iq = input->iq + phase * input->qq,
        .qi = q_scale * input->qi,
        .qq = q_scale * input->qq,
    };
    return output;
}

static bool quantize_signed_coefficient(double requested, double scale,
                                        int8_t *code, double *actual)
{
    if (!isfinite(requested) || !(scale > 0.0) || !isfinite(scale))
        return false;

    double unrounded = requested * scale;
    if (!isfinite(unrounded) ||
        unrounded < (double)INT8_MIN - 0.5 ||
        unrounded >= (double)INT8_MAX + 0.5)
        return false;

    long scaled = lround(unrounded);
    if (scaled < INT8_MIN || scaled > INT8_MAX)
        return false;

    *code = (int8_t)scaled;
    *actual = (double)*code / scale;
    return true;
}

static double wrap_pi(double angle)
{
    return atan2(sin(angle), cos(angle));
}

static bool circular_pair_mean(double a, double b, double *mean)
{
    double y = sin(a) + sin(b);
    double x = cos(a) + cos(b);

    if (hypot(x, y) < 1e-12)
        return false;
    *mean = atan2(y, x);
    return true;
}

static bool calculate_tx_qec(const struct measurement *positive,
                             const struct measurement *negative,
                             double rx_gain, double rx_phase,
                             double *gain, double *phase)
{
    struct correlation_matrix p =
        correct_rx_correlations(&positive->tx, rx_gain, rx_phase);
    struct correlation_matrix n =
        correct_rx_correlations(&negative->tx, rx_gain, rx_phase);

    // Naming follows the Tx-QEC derivation and the original Android code:
    // https://scalerf.com/cals/txqec.html
    double b  = p.ii + n.ii;
    double d  = p.iq + n.iq;
    double a  = p.qi + n.qi;
    double c  = p.qq + n.qq;
    double bp = p.ii - n.ii;
    double dp = p.iq - n.iq;
    double ap = p.qi - n.qi;
    double cp = p.qq - n.qq;

    double i_energy = a * a + dp * dp + b * b + cp * cp;
    double q_energy = d * d + ap * ap + c * c + bp * bp;
    if (!(i_energy > 0.0) || !(q_energy >= 0.0) ||
        !isfinite(i_energy) || !isfinite(q_energy))
        return false;

    double i_angle_1 = atan2(-a - dp, b - cp);
    double i_angle_2 = atan2(-a + dp, b + cp);
    double q_angle_1 = atan2(d + ap, c - bp);
    double q_angle_2 = atan2(d - ap, c + bp);
    double i_angle;
    double q_angle;

    if (!circular_pair_mean(i_angle_1, i_angle_2, &i_angle) ||
        !circular_pair_mean(q_angle_1, q_angle_2, &q_angle))
        return false;

    *gain = sqrt(q_energy / i_energy);
    *phase = wrap_pi(i_angle - q_angle);
    return isfinite(*gain) && isfinite(*phase);
}

static void calculate_nominal_measurements(
    struct measurement rx_positive[CHANNELS][CHANNELS],
    struct measurement rx_negative[CHANNELS][CHANNELS],
    struct measurement tx_positive[CHANNELS][CHANNELS],
    struct measurement tx_negative[CHANNELS][CHANNELS],
    struct quadrature_measurement measured[CHANNELS])
{
    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        const struct measurement *rx_p =
            &rx_positive[antenna][antenna];
        const struct measurement *rx_n =
            &rx_negative[antenna][antenna];
        const struct measurement *tx_p =
            &tx_positive[antenna][antenna];
        const struct measurement *tx_n =
            &tx_negative[antenna][antenna];
        double rx_correction_gain;
        double rx_correction_phase;

        measured[antenna].rx_gain = NAN;
        measured[antenna].rx_phase = NAN;
        measured[antenna].tx_gain = NAN;
        measured[antenna].tx_phase = NAN;

        measured[antenna].rx_valid = calculate_rx_metrics(
            rx_p, rx_n,
            &measured[antenna].rx_gain,
            &measured[antenna].rx_phase,
            &rx_correction_gain,
            &rx_correction_phase);

        // Remove the measured RX imbalance from the correlation matrices in
        // software so the reported TX terms characterize the TX path alone.
        // This does not quantize, program, or otherwise calculate a second set
        // of FPGA correction words.
        measured[antenna].tx_valid = measured[antenna].rx_valid &&
            calculate_tx_qec(tx_p, tx_n,
                             rx_correction_gain, rx_correction_phase,
                             &measured[antenna].tx_gain,
                             &measured[antenna].tx_phase);
    }
}

static void calculate_qec(
    struct measurement rx_positive[CHANNELS][CHANNELS],
    struct measurement rx_negative[CHANNELS][CHANNELS],
    struct measurement tx_positive[CHANNELS][CHANNELS],
    struct measurement tx_negative[CHANNELS][CHANNELS],
    bool enable_tx_qec,
    struct qec_coefficients qec[CHANNELS])
{
    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        const struct measurement *rx_p =
            &rx_positive[antenna][antenna];
        const struct measurement *rx_n =
            &rx_negative[antenna][antenna];
        const struct measurement *tx_p =
            &tx_positive[antenna][antenna];
        const struct measurement *tx_n =
            &tx_negative[antenna][antenna];

        qec[antenna].rx_gain = NAN;
        qec[antenna].rx_phase = NAN;
        qec[antenna].tx_gain = NAN;
        qec[antenna].tx_phase = NAN;
        qec[antenna].rx_gain_code = 0;
        qec[antenna].rx_phase_code = 0;
        qec[antenna].tx_gain_code = 0;
        qec[antenna].tx_phase_code = 0;

        double measured_rx_gain;
        double measured_rx_phase;
        double rx_gain;
        double rx_phase;
        // sdm_cic multiplies each signed 8-bit coefficient by a 9-bit sample
        // and rounds 17 bits back to 9, so the hardware coefficient is code/256.
        qec[antenna].rx_valid = calculate_rx_metrics(
            rx_p, rx_n, &measured_rx_gain, &measured_rx_phase,
            &rx_gain, &rx_phase) &&
            quantize_signed_coefficient(
                rx_gain, 256.0, &qec[antenna].rx_gain_code,
                &qec[antenna].rx_gain) &&
            quantize_signed_coefficient(
                rx_phase, 256.0, &qec[antenna].rx_phase_code,
                &qec[antenna].rx_phase);

        // TX imbalance is still measured and reported separately, but do not
        // derive FPGA TX correction terms unless the user explicitly opts in.
        if (!enable_tx_qec) {
            qec[antenna].tx_valid = false;
            continue;
        }

        double tx_gain_error;
        double tx_phase_error;
        qec[antenna].tx_valid = qec[antenna].rx_valid && calculate_tx_qec(
            tx_p, tx_n, qec[antenna].rx_gain, qec[antenna].rx_phase,
            &tx_gain_error, &tx_phase_error);

        if (qec[antenna].tx_valid) {
            double cosine = cos(tx_phase_error);
            if (fabs(cosine) <= 1e-12) {
                qec[antenna].tx_valid = false;
            } else {
                // Convert the Tx-QEC derivation's amplitude ratio and angle
                // into tx_path's triangular correction matrix.  Its 17-to-8
                // product round places the signed coefficient at code/512.
                double tx_gain = tx_gain_error / cosine - 1.0;
                double tx_phase = sin(tx_phase_error);

                qec[antenna].tx_valid = quantize_signed_coefficient(
                        tx_gain, 512.0, &qec[antenna].tx_gain_code,
                        &qec[antenna].tx_gain) &&
                    quantize_signed_coefficient(
                        tx_phase, 512.0, &qec[antenna].tx_phase_code,
                        &qec[antenna].tx_phase);
            }
        }
    }
}

static bool all_qec_valid(const struct qec_coefficients qec[CHANNELS],
                          bool require_tx_qec)
{
    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        if (!qec[antenna].rx_valid ||
            (require_tx_qec && !qec[antenna].tx_valid))
            return false;
    }
    return true;
}

static bool calculate_loopback_fit(const struct measurement *measurement,
                                   double *gain, double *rms_error)
{
    // Fit one complex coefficient H (gain and phase) to the centered receive
    // samples: RX = H*reference + residual.  The correlation below is
    // sum(RX*conj(reference)); its squared magnitude divided by reference
    // energy is the received energy explained by the fitted tone.
    double correlation_i = measurement->tx.ii + measurement->tx.qq;
    double correlation_q = measurement->tx.qi - measurement->tx.iq;
    double reference_energy = measurement->reference_energy;
    double received_energy = measurement->rx.ii + measurement->rx.qq;

    if (!(reference_energy > 0.0) || !(received_energy >= 0.0) ||
        !isfinite(reference_energy) || !isfinite(received_energy) ||
        !isfinite(correlation_i) || !isfinite(correlation_q))
        return false;

    double correlation_power =
        correlation_i * correlation_i + correlation_q * correlation_q;
    double tone_energy = correlation_power / reference_energy;
    if (!(tone_energy > 0.0) || !isfinite(tone_energy))
        return false;

    double residual_energy = received_energy - tone_energy;
    if (residual_energy < 0.0) {
        // Cauchy-Schwarz guarantees a nonnegative residual.  Permit only the
        // tiny negative value that can result from floating-point roundoff.
        double tolerance = 1e-9 * fmax(received_energy, tone_energy);
        if (residual_energy < -tolerance)
            return false;
        residual_energy = 0.0;
    }

    *gain = sqrt(correlation_power) / reference_energy;
    *rms_error = sqrt(residual_energy / tone_energy);
    return isfinite(*gain) && isfinite(*rms_error);
}

static void print_gain_results(
    struct measurement positive[CHANNELS][CHANNELS],
    const char *stage)
{
    printf("\nComplex channel magnitude H[RX][TX] (+tone, %s)\n", stage);
    puts(" RX  TX      gain       gain_dBFS   rms_error_%    error_dBc");

    for (unsigned rx = 0; rx < CHANNELS; rx++) {
        for (unsigned tx = 0; tx < CHANNELS; tx++) {
            double gain;
            double rms_error;
            if (!calculate_loopback_fit(&positive[rx][tx],
                                        &gain, &rms_error)) {
                printf(" %u   %u      invalid      invalid      invalid        invalid\n",
                       rx, tx);
                continue;
            }

            double gain_db = gain > 0.0 ? 20.0 * log10(gain) : -INFINITY;
            double error_db = rms_error > 0.0
                            ? 20.0 * log10(rms_error) : -INFINITY;

            printf(" %u   %u   %10.6f   %10.3f   %9.4f%%   %10.3f\n",
                   rx, tx, gain, gain_db, 100.0 * rms_error, error_db);
        }
    }

    puts("\nGain is the correlated RX complex amplitude relative to ADC full scale;");
    puts("it is not an absolute, power-calibrated RF S-parameter.");
    puts("RMS error is the residual after removing the fitted complex tone,");
    puts("relative to the fitted received-tone RMS; error_dBc is its power ratio.");
}

static void print_nominal_measurements(
    const struct quadrature_measurement measured[CHANNELS],
    const char *stage)
{
    printf("\nDirect-loopback quadrature correction coefficients (%s)\n",
           stage);
    puts(" ANT    RX_gain    RX_phase      TX_gain    TX_phase_rad");

    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        if (!measured[antenna].rx_valid) {
            printf("  %u     invalid     invalid      invalid         invalid\n",
                   antenna);
            continue;
        }
        if (!measured[antenna].tx_valid) {
            printf("  %u   %+9.6f  %+10.6f      invalid         invalid\n",
                   antenna, measured[antenna].rx_gain,
                   measured[antenna].rx_phase);
            continue;
        }

        printf("  %u   %+9.6f  %+10.6f   %+10.6f    %+12.6f\n",
               antenna, measured[antenna].rx_gain,
               measured[antenna].rx_phase, measured[antenna].tx_gain,
               measured[antenna].tx_phase);
    }
}

static void print_qec_results(const struct qec_coefficients qec[CHANNELS],
                              bool include_tx)
{
    if (!include_tx) {
        puts("\nDirect-loopback FPGA RX quadrature correction terms");
        puts(" ANT    RX_gain   RX_phase  RX_word");
        for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
            if (!qec[antenna].rx_valid) {
                printf("  %u     invalid    invalid   invalid\n", antenna);
                continue;
            }

            uint16_t rx_word = pack_rx_qec_word(
                qec[antenna].rx_gain_code, qec[antenna].rx_phase_code);
            printf("  %u   %+9.6f %+9.6f   0x%04x\n",
                   antenna, qec[antenna].rx_gain,
                   qec[antenna].rx_phase, rx_word);
        }
        puts("TX QEC correction is disabled; use -tx 1 to enable it.");
        return;
    }

    puts("\nDirect-loopback FPGA quadrature correction terms");
    puts(" ANT    RX_gain   RX_phase  RX_word     TX_gain   TX_phase  TX_codes(G/P)");
    for (unsigned antenna = 0; antenna < CHANNELS; antenna++) {
        if (!qec[antenna].rx_valid) {
            printf("  %u     invalid    invalid   invalid      invalid    invalid       invalid\n",
                   antenna);
            continue;
        }

        uint16_t rx_word = pack_rx_qec_word(
            qec[antenna].rx_gain_code, qec[antenna].rx_phase_code);

        if (!qec[antenna].tx_valid) {
            printf("  %u   %+9.6f %+9.6f   0x%04x      invalid    invalid       invalid\n",
                   antenna, qec[antenna].rx_gain,
                   qec[antenna].rx_phase, rx_word);
            continue;
        }

        printf("  %u   %+9.6f %+9.6f   0x%04x   %+9.6f %+9.6f      %+4d/%+4d\n",
               antenna, qec[antenna].rx_gain, qec[antenna].rx_phase,
               rx_word, qec[antenna].tx_gain, qec[antenna].tx_phase,
               (int)qec[antenna].tx_gain_code,
               (int)qec[antenna].tx_phase_code);
    }
}

int main(int argc, char **argv)
{
    int option_status = parse_options(argc, argv);
    if (option_status != 0)
        return option_status > 0 ? EXIT_SUCCESS : EXIT_FAILURE;

    int status = EXIT_FAILURE;
    int fd_rx = -1;
    int lease_held = 0;
    int radio_configured = 0;
    void *ring = MAP_FAILED;
    size_t map_len = 0;
    pthread_t tx_thread;
    int tx_thread_started = 0;

    struct measurement rx_positive[CHANNELS][CHANNELS] = {0};
    struct measurement rx_negative[CHANNELS][CHANNELS] = {0};
    struct measurement tx_positive[CHANNELS][CHANNELS] = {0};
    struct measurement tx_negative[CHANNELS][CHANNELS] = {0};
    struct qec_coefficients qec[CHANNELS] = {{0}};
    struct quadrature_measurement measured_before[CHANNELS] = {{0}};
    struct quadrature_measurement measured_after[CHANNELS] = {{0}};
    uint64_t rx_phase = 0;
    uint64_t rx_step = dds_step(g_config.tone_freq_hz, EXACT_RX_RATE);

    atomic_store(&g_requested_step,
                 dds_step(g_config.tone_freq_hz, EXACT_TX_RATE));
    atomic_store(&g_requested_polarity, 1);
    atomic_store(&g_requested_generation, 1u);

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    init_luts();

    fd_rx = open(RX_DEVICE, O_RDWR | O_NONBLOCK);
    if (fd_rx < 0) {
        perror("open " RX_DEVICE);
        goto cleanup;
    }

    if (jtag_acquire_lease(fd_rx) < 0) {
        perror("acquire JTAG lease");
        goto cleanup;
    }
    lease_held = 1;

    if (g_config.clear_qec_only) {
        if (ioctl(fd_rx, CSI_IOC_JTAG_SETUP) < 0 && errno != ENOTTY) {
            perror("CSI_IOC_JTAG_SETUP");
            goto cleanup;
        }
        if (program_qec(fd_rx, qec, true, true) < 0) {
            perror("clear FPGA QEC corrections");
            goto cleanup;
        }
        puts("Cleared FPGA RX and TX QEC corrections on antennas 0..3.");
        status = EXIT_SUCCESS;
        goto cleanup;
    }

    struct csi_ring_info ri;
    if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri) < 0) {
        perror("CSI_IOC_GET_RING_INFO");
        goto cleanup;
    }

    size_t page = (size_t)sysconf(_SC_PAGESIZE);
    map_len = ((size_t)ri.ring_size + page - 1) & ~(page - 1);
    ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd_rx, 0);
    if (ring == MAP_FAILED) {
        perror("mmap RX ring");
        goto cleanup;
    }

    if (setup_radio(fd_rx, &g_config) < 0) {
        perror("configure radio");
        radio_configured = 1; // ensure the partially enabled TX is shut down
        goto cleanup;
    }
    radio_configured = 1;

    if (program_qec(fd_rx, qec, true, true) < 0) {
        perror("clear FPGA QEC corrections");
        goto cleanup;
    }
    puts("Cleared FPGA RX and TX QEC corrections before measurement.");

    // Capture may already have been active during radio setup.  Empty the
    // ring before starting TX so it cannot overflow while TX comes online.
    if (discard_backlog(fd_rx, &rx_phase, rx_step) < 0) {
        perror("discard initial RX data");
        goto cleanup;
    }

    int thread_error = pthread_create(&tx_thread, NULL, tx_worker, NULL);
    if (thread_error != 0) {
        errno = thread_error;
        perror("start TX worker");
        goto cleanup;
    }
    tx_thread_started = 1;
    atomic_store(&g_radio_ready, true);

    // Wait until at least one valid DSI buffer has been queued, then allow it
    // time to reach scanout before beginning the first integration.
    for (int n = 0; n < 1000 && !g_stop; n++) {
        if (discard_backlog(fd_rx, &rx_phase, rx_step) < 0) {
            perror("drain RX during TX startup");
            goto cleanup;
        }
        if (atomic_load(&g_tx_streaming))
            break;
        if (atomic_load(&g_tx_failed)) {
            errno = EIO;
            perror("TX stream");
            goto cleanup;
        }
        usleep(1000);
    }
    if (!atomic_load(&g_tx_streaming)) {
        errno = ETIMEDOUT;
        perror("TX stream startup");
        goto cleanup;
    }

    // Keep draining during the short DSI scanout warm-up; sleeping here would
    // fill an 8 MiB RX ring in about 28 ms.
    for (int n = 0; n < 50 && !g_stop; n++) {
        usleep(1000);
        if (discard_backlog(fd_rx, &rx_phase, rx_step) < 0) {
            perror("drain RX during TX warm-up");
            goto cleanup;
        }
    }

    printf("QuadRF 4x4 MIMO loopback: RF %.3f MHz, tone %.3f MHz\n",
           g_config.rf_freq_mhz, g_config.tone_freq_hz / 1e6);
    printf("RX gain word %u, TX gain word %u, %u samples per path\n",
           g_config.rx_gain_word, g_config.tx_gain_word,
           g_config.integration_samples);
    printf("TX QEC correction: %s\n",
           g_config.tx_qec_enabled ? "enabled" : "disabled");
    fflush(stdout);

    if (capture_qec_cycle(
            fd_rx, ring, ri.ring_size, &rx_phase, rx_step, &g_config,
            "Initial measurements with QEC cleared",
            rx_positive, rx_negative, tx_positive, tx_negative) < 0)
        goto cleanup;

    calculate_nominal_measurements(
        rx_positive, rx_negative, tx_positive, tx_negative,
        measured_before);
    calculate_qec(rx_positive, rx_negative,
                  tx_positive, tx_negative, g_config.tx_qec_enabled, qec);
    print_gain_results(tx_positive, "before correction");
    print_nominal_measurements(measured_before, "before correction");
    print_qec_results(qec, g_config.tx_qec_enabled);

    if (!all_qec_valid(qec, g_config.tx_qec_enabled)) {
        fputs("\nQEC corrections were not applied: at least one estimate is "
              "invalid or outside the signed 8-bit coefficient range.\n",
              stderr);
        goto cleanup;
    }
    if (program_qec(fd_rx, qec, false, g_config.tx_qec_enabled) < 0) {
        perror("apply FPGA QEC corrections");
        goto cleanup;
    }
    if (g_config.tx_qec_enabled)
        puts("\nApplied RX and TX QEC corrections to FPGA antennas 0..3.");
    else
        puts("\nApplied RX QEC corrections to FPGA antennas 0..3; "
             "TX QEC correction remains disabled.");

    if (capture_qec_cycle(
            fd_rx, ring, ri.ring_size, &rx_phase, rx_step, &g_config,
            g_config.tx_qec_enabled
                ? "Verification measurements with RX and TX QEC applied"
                : "Verification measurements with RX QEC applied",
            rx_positive, rx_negative, tx_positive, tx_negative) < 0)
        goto cleanup;

    // Report residual imbalance only.  Do not derive, quantize, or apply a
    // second set of FPGA correction coefficients from the verification pass.
    calculate_nominal_measurements(
        rx_positive, rx_negative, tx_positive, tx_negative,
        measured_after);
    print_gain_results(tx_positive, "after correction");
    print_nominal_measurements(measured_after, "after correction");
    status = EXIT_SUCCESS;

cleanup:
    g_stop = 1;
    atomic_store(&g_radio_ready, true);
    if (tx_thread_started)
        pthread_join(tx_thread, NULL);
    if (radio_configured && fd_rx >= 0)
        radio_standby(fd_rx);
    if (lease_held)
        ioctl(fd_rx, CSI_IOC_JTAG_RELEASE_LEASE);
    if (ring != MAP_FAILED)
        munmap(ring, map_len);
    if (fd_rx >= 0)
        close(fd_rx);
    return status;
}
