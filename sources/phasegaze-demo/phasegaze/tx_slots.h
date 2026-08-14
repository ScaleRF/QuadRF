// tx_slots.h
// ESP32-C5 beacon slot frequencies and colors (see channel_cycle_chart.txt).

#ifndef TX_SLOTS_H
#define TX_SLOTS_H

#define TX_SLOT_COUNT        5
#define TX_FREQ_MARGIN_MHZ   10.0

typedef struct {
    int    slot;
    double center_mhz;
    unsigned char r, g, b;
} tx_slot_info_t;

extern const tx_slot_info_t tx_slots[TX_SLOT_COUNT];
extern const int tx_capture_order[TX_SLOT_COUNT];

const tx_slot_info_t *tx_slot_by_id(int slot_id);

#endif // TX_SLOTS_H
