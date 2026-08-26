// Front-end control CLI: talks to the CSI driver's JTAG ioctls to bring up and
// steer the two MAX285x transceivers.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "../drivers/csi/fpga_csi.h"
#include "max285x.h"

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s [options] --init\n"
        "  %s [options] --max2850\n"
        "  %s [options] --max2851\n"
        "  %s [options] --tx <spec>\n"
        "  %s [options] --rx <spec>\n"
        "  %s [options] --status <rx|tx>\n"
        "  %s [options] read  <addr>\n"
        "  %s [options] write <addr> <value>\n"
        "\n"
        "Initialization:\n"
        "  --init                 Initialize both chips and leave in standby\n"
        "  --max2850              Initialize TX chip only and leave in standby\n"
        "  --max2851              Initialize RX chip only and leave in standby\n"
        "\n"
        "TX control (MAX2850):\n"
        "  --tx <spec>            Enter TX mode and optionally program parameters.\n"
        "  --rx <spec>            Enter RX mode and optionally program parameters.\n"
        "  --tx off               Put TX chip into standby\n"
        "  --rx off               Put RX chip into standby\n"
        "                  <spec> is comma-separated key=value pairs:\n"
        "                           antennas=<mask>   0xF is all 4 antennas\n"
        "                           bw=<MHz>          tx: 20|40, rx: any > 0 (default: 40)\n"
        "                           freq=<MHz>        only if provided\n"
        "                           gain=<0..63>      TX default-capped at 40 until unlocked\n"
        "                           agc=<dBFS>        rx only: automatic gain control setpoint\n"
        "                           pol=<rhcp|lhcp>   rx only: polarization (default: rhcp)\n"
        "                           interleave=<1|0>  rx only: 4-channel interleaved mode (default: 0)\n"
        "\n"
        "Status Dump:\n"
        "  --status <rx|tx>       Print the full register state of the selected chip\n"
        "\n"
        "Advanced options:\n"
        "  -d, --device <path>    Device node (default: /dev/csi_stream0)\n"
        "      --no-setup         Do not call CSI_IOC_JTAG_SETUP\n"
        "      --keep             Do not call CSI_IOC_JTAG_RELEASE on exit\n"
        "  -h, --help             Show this help\n"
        "\n",
        argv0, argv0, argv0, argv0, argv0, argv0, argv0, argv0);
}

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static unsigned long parse_ul(const char *s, unsigned long max, const char *what)
{
    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (errno != 0 || end == s || (end && *end != '\0')) {
        fprintf(stderr, "Error: invalid %s '%s'\n", what, s);
        exit(2);
    }
    if (v > max) {
        fprintf(stderr, "Error: %s '%s' out of range (max %lu)\n", what, s, max);
        exit(2);
    }
    return v;
}

static long parse_sl(const char *s, long minv, long maxv, const char *what)
{
    errno = 0;
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (errno != 0 || end == s || (end && *end != '\0')) {
        fprintf(stderr, "Error: invalid %s '%s'\n", what, s);
        exit(2);
    }
    if (v < minv || v > maxv) {
        fprintf(stderr, "Error: %s '%s' out of range (%ld..%ld)\n", what, s, minv, maxv);
        exit(2);
    }
    return v;
}

static double parse_double(const char *s, const char *what)
{
    errno = 0;
    char *end = NULL;
    double v = strtod(s, &end);
    if (errno != 0 || end == s || (end && *end != '\0') || !isfinite(v)) {
        fprintf(stderr, "Error: invalid %s '%s'\n", what, s);
        exit(2);
    }
    return v;
}

/* ioctl forwarder */
int jtag_write_u16(int fd, uint8_t addr, uint16_t value)
{
    struct csi_jtag_reg r;
    memset(&r, 0, sizeof(r));
    r.addr  = addr;
    r.value = value;
    return ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r);
}

int jtag_read_u16(int fd, uint8_t addr, uint16_t *out_value)
{
    if (!out_value) {
        errno = EINVAL;
        return -1;
    }
    struct csi_jtag_reg r;
    memset(&r, 0, sizeof(r));
    r.addr = addr;
    if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) != 0) return -1;
    *out_value = r.value;
    return 0;
}

struct txrx_spec {
    bool off;

    bool bw_specified;
    double bw_mhz;       /* TX: 20/40; RX: any floating point value */

    bool freq_specified;
    double freq_mhz;

    bool gain_specified;
    long gain;           /* tx: 0..63 ; rx: int16 */

    bool ant_specified;
    unsigned long ant;   /* tx: 0..0xF ; rx: accepted but ignored */

    bool agc_specified;
    double agc_dbfs;     /* rx only: dBFS level for threshold */

    bool pol_specified;
    bool pol_rhcp;       /* rx only: true for RHCP, false for LHCP */

    bool interleave_specified;
    bool interleave_on;  /* rx only: 4-channel interleaved mode */

    bool autosteer_specified; 
    bool autosteer_on;

    bool tx_follow_rx_specified;
    bool tx_follow_rx_on;
    
    bool tone_en_specified;   
    bool tone_en;
    
    bool tone_freq_specified; 
    double tone_freq_mhz;
    
    bool phase_specified[4];  
    double phase_deg[4];
};

static void spec_init(struct txrx_spec *s)
{
    memset(s, 0, sizeof(*s));
}

/* parse "off" OR "k=v,k=v" in a single argument */
static void parse_txrx_spec(const char *arg, bool is_tx, struct txrx_spec *out)
{
    spec_init(out);

    if (!arg || !*arg) {
        fprintf(stderr, "Error: missing %s spec\n", is_tx ? "tx" : "rx");
        exit(2);
    }

    if (strcmp(arg, "off") == 0) {
        out->off = true;
        return;
    }

    char *tmp = strdup(arg);
    if (!tmp) {
        fprintf(stderr, "Error: OOM\n");
        exit(2);
    }

    for (char *tok = strtok(tmp, ","); tok; tok = strtok(NULL, ",")) {
        while (*tok == ' ' || *tok == '\t') tok++;

        char *eq = strchr(tok, '=');
        if (!eq) {
            fprintf(stderr, "Error: bad %s token '%s' (expected key=value)\n",
                    is_tx ? "tx" : "rx", tok);
            free(tmp);
            exit(2);
        }

        *eq = '\0';
        const char *key = tok;
        const char *val = eq + 1;

        if (strcmp(key, "bw") == 0) {
            if (is_tx) {
                int bw = (int)parse_ul(val, 40, "bw");
                if (bw != 20 && bw != 40) {
                    fprintf(stderr, "Error: tx bw must be 20 or 40\n");
                    free(tmp);
                    exit(2);
                }
                out->bw_specified = true;
                out->bw_mhz = (double)bw;
            } else {
                double bw = parse_double(val, "bw");
                if (bw <= 0.0) {
                    fprintf(stderr, "Error: rx bw must be > 0\n");
                    free(tmp);
                    exit(2);
                }
                out->bw_specified = true;
                out->bw_mhz = bw;
            }
        } else if (strcmp(key, "freq") == 0) {
            out->freq_specified = true;
            out->freq_mhz = parse_double(val, "freq");
            if ((out->freq_mhz < 4900.0) || out->freq_mhz > 6000.0) {
                fprintf(stderr, "Carrier frequency must be between 4900 and 6000 (MHz)\n");
                free(tmp);
                exit(2);
            }
        } else if (strcmp(key, "gain") == 0) {
            out->gain_specified = true;
            if (is_tx) out->gain = (long)parse_ul(val, 63, "tx_gain");
            else       out->gain = (long)parse_sl(val, -32768, 32767, "rx_gain");
        } else if (strcmp(key, "antennas") == 0 || strcmp(key, "channels") == 0) {
            out->ant_specified = true;
            out->ant = parse_ul(val, 0xFul, "antennas");
        } else if (strcmp(key, "agc") == 0) {
            if (is_tx) {
                fprintf(stderr, "Error: agc is only supported for rx\n");
                free(tmp);
                exit(2);
            }
            out->agc_specified = true;
            out->agc_dbfs = parse_double(val, "agc");
        } else if (strcmp(key, "pol") == 0) {
            if (is_tx) {
                fprintf(stderr, "Error: pol is only supported for rx\n");
                free(tmp);
                exit(2);
            }
            out->pol_specified = true;
            if (strcasecmp(val, "rhcp") == 0) out->pol_rhcp = true;
            else if (strcasecmp(val, "lhcp") == 0) out->pol_rhcp = false;
            else {
                fprintf(stderr, "Error: rx pol must be 'rhcp' or 'lhcp'\n");
                free(tmp);
                exit(2);
            }
        } else if (strcmp(key, "interleave") == 0) {
            if (is_tx) {
                fprintf(stderr, "Error: interleave is only supported for rx\n");
                free(tmp);
                exit(2);
            }
            out->interleave_specified = true;
            if (strcmp(val, "1") == 0 || strcasecmp(val, "on") == 0) out->interleave_on = true;
            else if (strcmp(val, "0") == 0 || strcasecmp(val, "off") == 0) out->interleave_on = false;
            else {
                fprintf(stderr, "Error: rx interleave must be 1, 0, on, or off\n");
                free(tmp);
                exit(2);
            }
        } else if (strcmp(key, "autosteer") == 0) {
            out->autosteer_specified = true;
            out->autosteer_on = (parse_ul(val, 1, "autosteer") != 0);
        } else if (strcmp(key, "tx_follow_rx") == 0) {
            out->tx_follow_rx_specified = true;
            out->tx_follow_rx_on = (parse_ul(val, 1, "tx_follow_rx") != 0);
        } else if (strcmp(key, "tone_en") == 0) {
            out->tone_en_specified = true;
            out->tone_en = (parse_ul(val, 1, "tone_en") != 0);
        } else if (strcmp(key, "tone_freq") == 0) {
            out->tone_freq_specified = true;
            out->tone_freq_mhz = parse_double(val, "tone_freq");
        } else if (strcmp(key, "p1") == 0) {
            out->phase_specified[0] = true; out->phase_deg[0] = parse_double(val, "p1");
        } else if (strcmp(key, "p2") == 0) {
            out->phase_specified[1] = true; out->phase_deg[1] = parse_double(val, "p2");
        } else if (strcmp(key, "p3") == 0) {
            out->phase_specified[2] = true; out->phase_deg[2] = parse_double(val, "p3");
        } else if (strcmp(key, "p4") == 0) {
            out->phase_specified[3] = true; out->phase_deg[3] = parse_double(val, "p4");
        } else {
            fprintf(stderr, "Error: unknown %s key '%s'\n", is_tx ? "tx" : "rx", key);
            free(tmp);
            exit(2);
        }
    }

    free(tmp);
}

static int calibrate_dsi_link(int fd)
{
    printf("Calibrating DSI link timing...\n");
    uint16_t errors[9];
    uint16_t val1, val2;

    // First reset the phase register for consistency
    if (jtag_write_u16(fd, 0x33, 0x04) != 0) return -1;
    usleep(100);
    if (jtag_write_u16(fd, 0x33, 0x00) != 0) return -1;
    
    /* Sweep from tap 0 to 8 */
    for (int i = 0; i <= 8; i++) {
        /* Read CRC error count, wait ~50ms, read again */
        if (jtag_read_u16(fd, 0x35, &val1) != 0) return -1;
        usleep(50000); 
        if (jtag_read_u16(fd, 0x35, &val2) != 0) return -1;
        
        errors[i] = val2 - val1; /* Unsigned math handles 16-bit rollover safely */
        printf("  Tap %d: %u errors\n", i, errors[i]);
        
        /* Increment delay hardware tap if not at the end */
        if (i < 8) {
            if (jtag_write_u16(fd, 0x33, 0x03) != 0) return -1;
            if (jtag_write_u16(fd, 0x33, 0x02) != 0) return -1;
        }
    }
    
    /* Analyze results to find the widest safe window */
    int max_len = 0, current_len = 0;
    int best_start = 0, current_start = 0;
    
    for (int i = 0; i <= 8; i++) {
        if (errors[i] == 0) {
            if (current_len == 0) current_start = i;
            current_len++;
            if (current_len > max_len) {
                max_len = current_len;
                best_start = current_start;
            }
        } else {
            current_len = 0;
        }
    }
    
    int best_tap;
    if (max_len == 0) {
        fprintf(stderr, "Warning: No error-free DSI taps found! Defaulting to tap 4.\n");
        best_tap = 4;
    } else {
        /* Find the center of the safe window */
        best_tap = best_start + (max_len / 2);
        printf("Selected safest tap: %d (center of %d error-free taps)\n", best_tap, max_len);
    }
    
    /* The hardware is currently left at tap 8. Step backwards to the selected best tap. */
    for (int i = 8; i > best_tap; i--) {
        if (jtag_write_u16(fd, 0x33, 0x01) != 0) return -1;
        if (jtag_write_u16(fd, 0x33, 0x00) != 0) return -1;
    }
    
    return 0;
}


int main(int argc, char **argv)
{
    const char *dev = "/dev/csi_stream0";
    bool do_setup = true;
    bool do_release = true;

    bool do_init_both = false;
    bool do_init_2850 = false;
    bool do_init_2851 = false;

    bool do_tx = false;
    bool do_rx = false;
    
    bool do_status = false;
    bool status_rx = false;
    
    struct txrx_spec txs, rxs;
    spec_init(&txs);
    spec_init(&rxs);

    static const struct option long_opts[] = {
        {"device",   required_argument, 0, 'd'},
        {"no-setup", no_argument,       0,  1 },
        {"keep",     no_argument,       0,  2 },
        {"init",     no_argument,       0,  3 },
        {"max2850",  no_argument,       0,  4 },
        {"max2851",  no_argument,       0,  5 },
        {"tx",       required_argument, 0,  6 },
        {"rx",       required_argument, 0,  7 },
        {"status",   required_argument, 0,  8 },
        {"help",     no_argument,       0, 'h'},
        {"hs0", required_argument, 0, 0},
        {"hs1", required_argument, 0, 0},
        {0,0,0,0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:h", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            dev = optarg;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        case 1: /* --no-setup */
            do_setup = false;
            break;
        case 2: /* --keep */
            do_release = false;
            break;
        case 3: /* --init */
            do_init_both = true;
            break;
        case 4: /* --max2850 */
            do_init_2850 = true;
            break;
        case 5: /* --max2851 */
            do_init_2851 = true;
            break;
        case 6: /* --tx <spec> */
            do_tx = true;
            parse_txrx_spec(optarg, true, &txs);
            break;
        case 7: /* --rx <spec> */
            do_rx = true;
            parse_txrx_spec(optarg, false, &rxs);
            break;
        case 8: /* --status <rx|tx> */
            do_status = true;
            if (strcmp(optarg, "rx") == 0) {
                status_rx = true;
            } else if (strcmp(optarg, "tx") == 0) {
                status_rx = false;
            } else {
                fprintf(stderr, "Error: --status requires 'rx' or 'tx'\n");
                return 2;
            }
            break;
        default:
            usage(argv[0]);
            return 2;
        }
    }

    bool is_read = false, is_write = false;
    unsigned long rw_addr = 0, rw_val = 0;

    const bool any_action = (do_init_both || do_init_2850 || do_init_2851 || do_tx || do_rx || do_status);

    if (!any_action) {
        if (optind >= argc) {
            usage(argv[0]);
            return 2;
        }

        const char *cmd = argv[optind++];

        if (strcmp(cmd, "read") == 0 || strcmp(cmd, "r") == 0) {
            is_read = true;
            if (optind != argc - 0) { /* expect exactly one arg remaining */
                if (optind + 1 != argc) { usage(argv[0]); return 2; }
            }
            rw_addr = parse_ul(argv[optind++], 0xFFul, "addr");
        } else if (strcmp(cmd, "write") == 0 || strcmp(cmd, "w") == 0) {
            is_write = true;
            if (optind + 2 != argc) { usage(argv[0]); return 2; }
            rw_addr = parse_ul(argv[optind++], 0xFFul, "addr");
            rw_val  = parse_ul(argv[optind++], 0xFFFFul, "value");
        } else {
            fprintf(stderr, "Error: unknown command '%s'\n", cmd);
            usage(argv[0]);
            return 2;
        }

        if (optind != argc) { usage(argv[0]); return 2; }
    } else {
        if (optind != argc) {
            fprintf(stderr, "Error: unexpected positional arguments\n");
            usage(argv[0]);
            return 2;
        }
    }

    /* no redundant init modes */
    if (do_init_both && (do_init_2850 || do_init_2851)) {
        fprintf(stderr, "Error: use either --init OR --max2850/--max2851 (not both)\n");
        return 2;
    }

    if (!(do_init_both || do_init_2850 || do_init_2851 || do_tx || do_rx || do_status || is_read || is_write)) {
        usage(argv[0]);
        return 2;
    }

    int fd = open(dev, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "Error: open(%s) failed: %s\n", dev, strerror(errno));
        return 1;
    }

    int rc = 0;

    if (do_setup) {
        if (ioctl(fd, CSI_IOC_JTAG_SETUP) != 0) {
            fprintf(stderr, "Error: CSI_IOC_JTAG_SETUP failed: %s\n", strerror(errno));
            rc = 1;
            goto out;
        }
    }

    /* --- Acquire Exclusive Lease --- */
    uint64_t start_ns = now_ns();
    
    while (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) != 0) {
        if (errno == ENOTTY) {
            // Kernel driver is older and doesn't support leases. Safe to ignore.
            break;
        } else if (errno == EBUSY) {
            if (now_ns() - start_ns > 100000000ULL) { // 100ms timeout
                fprintf(stderr, "Error: JTAG hardware is locked by another process (100ms timeout).\n");
                rc = 1;
                goto out;
            }
            usleep(1000); // Sleep 1ms and try again
        } else {
            fprintf(stderr, "Warning: Failed to acquire JTAG lease: %s\n", strerror(errno));
            break;
        }
    }
    /* init paths */
    if (do_init_both) {

        if (calibrate_dsi_link(fd) != 0) { 
            fprintf(stderr, "Error: calibrate_dsi_link failed: %s\n", strerror(errno)); 
            rc = 1; goto out_release; 
        }

        if (max2850_init(fd) != 0) { fprintf(stderr, "Error: max2850_init failed: %s\n", strerror(errno)); rc = 1; goto out_release; }
        if (max2851_init(fd) != 0) { fprintf(stderr, "Error: max2851_init failed: %s\n", strerror(errno)); rc = 1; goto out_release; }
    }
    if (do_init_2850) {
        if (max2850_init(fd) != 0) { fprintf(stderr, "Error: max2850_init failed: %s\n", strerror(errno)); rc = 1; goto out_release; }
    }
    if (do_init_2851) {
        if (max2851_init(fd) != 0) { fprintf(stderr, "Error: max2851_init failed: %s\n", strerror(errno)); rc = 1; goto out_release; }
    }

    /* TX */
    if (do_tx) {
        int bw = txs.bw_specified ? (int)txs.bw_mhz : 40;

        if (txs.off) {
            if (max2850_set_idle(fd, bw) != 0) {
                fprintf(stderr, "Error: --tx off failed: %s\n", strerror(errno));
                rc = 1;
                goto out_release;
            }
        } else {
            if (max2850_tx_on(fd,
                              bw,
                              txs.freq_specified, txs.freq_mhz,
                              txs.gain_specified, (uint16_t)(txs.gain & 0xFFFF),
                              txs.ant_specified,  (uint16_t)(txs.ant & 0xFFFF)) != 0) {
                fprintf(stderr, "Error: --tx failed: %s\n", strerror(errno));
                rc = 1;
                goto out_release;
            }
        }

        /* Tx Tone Enable (0x26) */
        if (txs.tone_en_specified) {
            uint16_t val_26 = 0;
            jtag_read_u16(fd, 0x26, &val_26);
            if (txs.tone_en) val_26 |= 0x01;
            else val_26 &= ~0x01;
            jtag_write_u16(fd, 0x26, val_26);
        }

        /* Tone frequency (0x2F) - Shared with Rx */
        if (txs.tone_freq_specified) {
            uint16_t k_reg = 0;
            jtag_read_u16(fd, 0x27, &k_reg);
            double fs = (k_reg > 0) ? (352.0 / k_reg) : 88.0;
            double n_f = txs.tone_freq_mhz * 65536.0 / fs;
            jtag_write_u16(fd, 0x2F, (uint16_t)(int16_t)round(n_f));
        }        
        if (txs.tx_follow_rx_specified) {
            jtag_write_u16(fd, 0x6D, txs.tx_follow_rx_on ? 0x01 : 0x00);
        }

	/* Tx Phases (0x2C, 0x2D) */
        if (txs.phase_specified[0] || txs.phase_specified[1]) {
            uint16_t val_2c = 0;
            jtag_read_u16(fd, 0x2C, &val_2c);
            if (txs.phase_specified[0]) {
                uint8_t p = (uint8_t)round((txs.phase_deg[0] / 360.0) * 256.0);
                val_2c = (uint16_t)((val_2c & 0x00FF) | (p << 8));
            }
            if (txs.phase_specified[1]) {
                uint8_t p = (uint8_t)round((txs.phase_deg[1] / 360.0) * 256.0);
                val_2c = (uint16_t)((val_2c & 0xFF00) | p);
            }
            jtag_write_u16(fd, 0x2C, val_2c);
        }
        if (txs.phase_specified[2] || txs.phase_specified[3]) {
            uint16_t val_2d = 0;
            jtag_read_u16(fd, 0x2D, &val_2d);
            if (txs.phase_specified[2]) {
                uint8_t p = (uint8_t)round((txs.phase_deg[2] / 360.0) * 256.0);
                val_2d = (uint16_t)((val_2d & 0x00FF) | (p << 8));
            }
            if (txs.phase_specified[3]) {
                uint8_t p = (uint8_t)round((txs.phase_deg[3] / 360.0) * 256.0);
                val_2d = (uint16_t)((val_2d & 0xFF00) | p);
            }
            jtag_write_u16(fd, 0x2D, val_2d);
        }
    }

    /* RX */
    if (do_rx) {
        double target_bw = rxs.bw_specified ? rxs.bw_mhz : 40.0;
        
        /* Calculate digital filter register k */
        int k = (int)round(240.0 / target_bw);
        if (k < 5) k = 5;
        if (k > 63) k = 63;
        
        double actual_bw = 240.0 / k;
        int analog_bw = (k > 11) ? 20 : 40;

        /* Default to 4 antennas enabled (Rx1..Rx4) if not specified */
        uint8_t rx_mask = rxs.ant_specified ? (uint8_t)(rxs.ant) : 0x0Fu; // default Rx1..Rx4    

        if (rxs.off) {
            if (max2851_rx_off(fd, analog_bw) != 0) {
                fprintf(stderr, "Error: --rx off failed: %s\n", strerror(errno));
                rc = 1;
                goto out_release;
            }
        } else {
            if (max2851_rx_on(fd, analog_bw, rxs.freq_specified, rxs.freq_mhz, rxs.gain_specified, (int16_t)rxs.gain, true, rx_mask) != 0) {
                fprintf(stderr, "Error: --rx failed: %s\n", strerror(errno));
                rc = 1;
                goto out_release;
            }
            
            if(rxs.bw_specified){
                /* Program the calculated digital filter value to register 0x27 */
                if (jtag_write_u16(fd, 0x27, (uint16_t)k) != 0) {
                    fprintf(stderr, "Error: failed to write digital filter bandwidth to 0x27\n");
                    rc = 1;
                    goto out_release;
                }

                printf("RX Digital Filter: k=%d, actual_bw=%.2f MHz (Analog BW set to %d MHz)\n", 
                    k, actual_bw, analog_bw);
            }

            if (rxs.agc_specified) {
                /* Calculate threshold: dBFS = 20 * log10(thr / 180) -> thr = 180 * 10^(dBFS/20) */
                double thr_f = 180.0 * pow(10.0, rxs.agc_dbfs / 20.0);
                
                /* Clamp threshold safely */
                if (thr_f < 0.0) thr_f = 0.0;
                if (thr_f > 180.0) thr_f = 180.0;
                uint16_t thr = (uint16_t)round(thr_f);

                /* Enable AGC in register 0x6A */
                if (jtag_write_u16(fd, 0x6A, 0x0080) != 0) {
                    fprintf(stderr, "Error: failed to write AGC enable to 0x6A\n");
                    rc = 1;
                    goto out_release;
                }

                /* Write linear scale threshold to register 0x6B */
                if (jtag_write_u16(fd, 0x6B, thr) != 0) {
                    fprintf(stderr, "Error: failed to write AGC threshold to 0x6B\n");
                    rc = 1;
                    goto out_release;
                }

                printf("RX AGC Enabled: target_dBFS=%.2f, threshold_val=%u\n", rxs.agc_dbfs, thr);
            }

            if (rxs.pol_specified) {
                uint16_t val = rxs.pol_rhcp ? 0x01 : 0x00;
                if (jtag_write_u16(fd, 0x24, val) != 0) {
                    fprintf(stderr, "Error: failed to write polarization to 0x24\n");
                    rc = 1; goto out_release;
                }
                printf("RX Polarization: %s\n", rxs.pol_rhcp ? "RHCP" : "LHCP");
            }

            if (rxs.interleave_specified) {
                uint16_t val = rxs.interleave_on ? 0x01 : 0x00;
                if (jtag_write_u16(fd, 0x25, val) != 0) {
                    fprintf(stderr, "Error: failed to write interleave mode to 0x25\n");
                    rc = 1; goto out_release;
                }
                printf("RX Interleaved Mode: %s\n", rxs.interleave_on ? "ON" : "OFF");
            }

/* Rx Auto Steer & Tone Enable (0x2E) */
            if (rxs.autosteer_specified || rxs.tone_en_specified) {
                uint16_t val_2e = 0;
                jtag_read_u16(fd, 0x2E, &val_2e);
                if (rxs.autosteer_specified) {
                    if (rxs.autosteer_on) val_2e |= 0x01; else val_2e &= ~0x01;
                }
                if (rxs.tone_en_specified) {
                    if (rxs.tone_en) val_2e |= 0x02; else val_2e &= ~0x02;
                }
                jtag_write_u16(fd, 0x2E, val_2e);
            }

            /* Tone frequency (0x2F) - Shared with Tx */
            if (rxs.tone_freq_specified) {
                uint16_t k_reg = 0;
                jtag_read_u16(fd, 0x27, &k_reg);
                double fs = (k_reg > 0) ? (352.0 / k_reg) : 88.0;
                double n_f = rxs.tone_freq_mhz * 65536.0 / fs;
                jtag_write_u16(fd, 0x2F, (uint16_t)(int16_t)round(n_f));
            }            /* Rx Phases (0x2A, 0x2B) */
            if (rxs.phase_specified[0] || rxs.phase_specified[1]) {
                uint16_t val_2a = 0;
                jtag_read_u16(fd, 0x2A, &val_2a);
                if (rxs.phase_specified[0]) {
                    uint8_t p = (uint8_t)round((rxs.phase_deg[0] / 360.0) * 256.0);
                    val_2a = (uint16_t)((val_2a & 0x00FF) | (p << 8));
                }
                if (rxs.phase_specified[1]) {
                    uint8_t p = (uint8_t)round((rxs.phase_deg[1] / 360.0) * 256.0);
                    val_2a = (uint16_t)((val_2a & 0xFF00) | p);
                }
                jtag_write_u16(fd, 0x2A, val_2a);
            }
            if (rxs.phase_specified[2] || rxs.phase_specified[3]) {
                uint16_t val_2b = 0;
                jtag_read_u16(fd, 0x2B, &val_2b);
                if (rxs.phase_specified[2]) {
                    uint8_t p = (uint8_t)round((rxs.phase_deg[2] / 360.0) * 256.0);
                    val_2b = (uint16_t)((val_2b & 0x00FF) | (p << 8));
                }
                if (rxs.phase_specified[3]) {
                    uint8_t p = (uint8_t)round((rxs.phase_deg[3] / 360.0) * 256.0);
                    val_2b = (uint16_t)((val_2b & 0xFF00) | p);
                }
                jtag_write_u16(fd, 0x2B, val_2b);
            }
        }
    }

    /* Status Output */
    if (do_status) {
        if (status_rx) {
            if (max2851_status(fd) != 0) {
                fprintf(stderr, "Error: --status rx failed\n");
                rc = 1;
                goto out_release;
            }
        } else {
            if (max2850_status(fd) != 0) {
                fprintf(stderr, "Error: --status tx failed\n");
                rc = 1;
                goto out_release;
            }
	    uint16_t tx_follow_rx_val = 0;
            if (jtag_read_u16(fd, 0x6D, &tx_follow_rx_val) == 0) {
                printf("- Tx follow Rx: %s\n", (tx_follow_rx_val & 0x01) ? "ON" : "OFF");
            }
        }
    }

    /* Generic read/write passthrough (FPGA register space) */
    if (is_read) {
        uint16_t v = 0;
        if (jtag_read_u16(fd, (uint8_t)rw_addr, &v) != 0) {
            fprintf(stderr, "Error: read addr=0x%02lX failed: %s\n", rw_addr, strerror(errno));
            rc = 1;
            goto out_release;
        }
        printf("READ  addr=0x%02lX -> value=0x%04X\n", rw_addr, (unsigned)v);
    } else if (is_write) {
        if (jtag_write_u16(fd, (uint8_t)rw_addr, (uint16_t)rw_val) != 0) {
            fprintf(stderr, "Error: write addr=0x%02lX value=0x%04lX failed: %s\n",
                    rw_addr, rw_val, strerror(errno));
            rc = 1;
            goto out_release;
        }
        printf("WRITE addr=0x%02lX <- value=0x%04lX [OK]\n", rw_addr, rw_val);
    }

out_release:

    ioctl(fd, CSI_IOC_JTAG_RELEASE_LEASE);

    if (do_setup && do_release) {
        if (ioctl(fd, CSI_IOC_JTAG_RELEASE) != 0) {
            fprintf(stderr, "Warning: CSI_IOC_JTAG_RELEASE failed: %s\n", strerror(errno));
        }
    }

out:
    close(fd);
    return rc;
}
