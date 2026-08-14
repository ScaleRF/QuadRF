// ring_buffer.c
// Ring buffer operations for CSI data acquisition

#include "ring_buffer.h"
#include "utils.h"
#include "config.h"

#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#include "fpga_csi.h"

void consume_bytes(int fd, uint32_t nbytes)
{
    if (!nbytes) return;
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &nbytes) < 0)
        die("CSI_IOC_CONSUME_BYTES");
}

void ring_wait_and_get_one_block_fast(int fd, const void *ring, uint64_t ring_size,
                                      uint8_t *dst, read_telem_t *rt)
{
    int spins = WAIT_SPIN_ITERS;

    while (1)
    {
        uint64_t t0 = now_ns();
        struct csi_ring_info ri;
        if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
            die("CSI_IOC_GET_RING_INFO");
        uint64_t t1 = now_ns();

        rt->ns_getinfo += (t1 - t0);
        rt->calls_getinfo++;

        uint32_t used = ring_used_bytes(ri.head, ri.tail, ring_size);
        used -= (used % BYTES_PER_FRAME);

        uint32_t max_keep = (uint32_t)(MAX_QUEUED_BLOCKS * BLOCK_BYTES);
        if (used > max_keep)
        {
            uint32_t drop = used - max_keep;
            drop -= (drop % BYTES_PER_FRAME);
            if (drop)
            {
                uint64_t tc0 = now_ns();
                consume_bytes(fd, drop);
                uint64_t tc1 = now_ns();
                rt->ns_consume += (tc1 - tc0);
                rt->calls_consume++;
            }
            spins = WAIT_SPIN_ITERS;
            continue;
        }

        if (used >= BLOCK_BYTES)
        {
            uint32_t tail = ri.tail;

            uint32_t n1 = BLOCK_BYTES;
            if ((uint64_t)tail + (uint64_t)n1 > ring_size)
            {
                n1 = (uint32_t)ring_size - tail;
                rt->wraps++;
            }

            uint64_t tm0 = now_ns();
            memcpy(dst, (const uint8_t*)ring + tail, n1);
            uint32_t rem = BLOCK_BYTES - n1;
            if (rem) memcpy(dst + n1, (const uint8_t*)ring, rem);
            uint64_t tm1 = now_ns();
            rt->ns_copy += (tm1 - tm0);

            uint64_t tc0 = now_ns();
            consume_bytes(fd, BLOCK_BYTES);
            uint64_t tc1 = now_ns();
            rt->ns_consume += (tc1 - tc0);
            rt->calls_consume++;

            return;
        }

        if (spins-- > 0) cpu_relax();
        else usleep(WAIT_SLEEP_US);
    }
}

