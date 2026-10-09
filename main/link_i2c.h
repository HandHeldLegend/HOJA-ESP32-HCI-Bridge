#ifndef LINK_I2C_H
#define LINK_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "drivers/bluetooth/hlink.h"

// Also the answer to the old version request, so new RP2040 firmware knows it's the bridge
#define BRIDGE_FW_VERSION 0xB003

typedef struct
{
    uint32_t transactions;
    uint32_t writes;
    uint32_t reads;
    uint32_t short_writes;
    uint32_t legacy_probes;
    uint32_t reinits;
} link_i2c_stats_s;

// rx_cb runs in the interrupt
void link_i2c_init(hlink_rx_cb_t rx_cb, void *ctx, uint8_t session);

// Safe from any task, waits up to wait_ms for room
bool link_i2c_send(uint8_t channel, const uint8_t *data, uint16_t len, uint32_t wait_ms);
bool link_i2c_send2(uint8_t channel, const uint8_t *hdr, uint16_t hdr_len,
                    const uint8_t *body, uint16_t body_len, uint32_t wait_ms);

// Call periodically
void link_i2c_watchdog(uint32_t now_ms);

// How long since the RP2040 last finished a transfer, bus resets don't count
uint32_t link_i2c_peer_quiet_ms(uint32_t now_ms);

void link_i2c_get_stats(hlink_stats_s *link, link_i2c_stats_s *bus);

#endif
