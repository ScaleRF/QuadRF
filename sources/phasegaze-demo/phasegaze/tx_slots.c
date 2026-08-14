// tx_slots.c

#include <stddef.h>
#include "tx_slots.h"

const tx_slot_info_t tx_slots[TX_SLOT_COUNT] = {
    { 0, 5180.0, 38, 64,  0 },
    { 1, 5320.0,  0, 64,  6 },
    { 2, 5540.0,  0, 51, 64 },
    { 3, 5700.0,  0,  0, 64 },
    { 4, 5885.0, 59,  0, 64 },
};

const int tx_capture_order[TX_SLOT_COUNT] = { 2, 3, 4, 0, 1 };

const tx_slot_info_t *tx_slot_by_id(int slot_id)
{
    for (int i = 0; i < TX_SLOT_COUNT; ++i) {
        if (tx_slots[i].slot == slot_id)
            return &tx_slots[i];
    }
    return NULL;
}
