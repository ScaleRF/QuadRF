// ring_buffer.h
// Ring buffer operations and telemetry for CSI data acquisition

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include "config.h"

// ------------------------------------------------------------
// Read Telemetry Structure
// ------------------------------------------------------------

typedef struct {
    uint64_t ns_getinfo;
    uint64_t ns_copy;
    uint64_t ns_consume;
    uint64_t calls_getinfo;
    uint64_t calls_consume;
    uint64_t wraps;
} read_telem_t;

// ------------------------------------------------------------
// Inline Ring Buffer Utilities
// ------------------------------------------------------------

static inline uint32_t ring_used_bytes(uint32_t head, uint32_t tail, uint64_t ring_size)
{
    return (head >= tail) ? (head - tail)
                          : ((uint32_t)ring_size - (tail - head));
}

// ------------------------------------------------------------
// Ring Buffer Functions (implemented in ring_buffer.c)
// ------------------------------------------------------------

void consume_bytes(int fd, uint32_t nbytes);

// Wait until at least one block is available. If backlog is large, drop oldest blocks.
// Optimized for fewer ioctls:
//   - exactly one GET_RING_INFO per returned block
//   - exactly one CONSUME_BYTES per returned block
void ring_wait_and_get_one_block_fast(int fd, const void *ring, uint64_t ring_size,
                                      uint8_t *dst, read_telem_t *rt);

#endif // RING_BUFFER_H

