// signal_processing.c
// Signal processing: FFT conversion and LO programming

#include "signal_processing.h"
#include "config.h"

#include <math.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#include "fpga_csi.h"

void cs8_to_fftw_ch(const int8_t *src, int ch, fftwf_complex *dst)
{
    for (int n = 0; n < FFT_SIZE; ++n)
    {
        size_t base = (size_t)n * BYTES_PER_FRAME + (size_t)ch * BYTES_PER_IQ;
        dst[n][0] = (float)src[base + 0] / 127.0f;
        dst[n][1] = (float)src[base + 1] / 127.0f;
    }
}

int program_set_freq(int fd, double freq_mhz)
{
    double ratio = freq_mhz / 80.0;
    int idiv = (int)floor(ratio);
    int fdiv = (int)llround((ratio - idiv) * (double)(1u << 20));

    struct csi_jtag_reg r;
    r.addr = 0x43;

    r.value = (uint16_t)((15u<<10) | (1u<<9) | (idiv & 0x7f));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    r.value = (uint16_t)((16u<<10) | ((fdiv >> 10) & 0x3ff));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    r.value = (uint16_t)((17u<<10) | (fdiv & 0x3ff));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    return 0;
}

