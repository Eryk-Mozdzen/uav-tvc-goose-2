#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    COMMUNICATION_EVENT_RX_HALF,
    COMMUNICATION_EVENT_RX_CPLT,
    COMMUNICATION_EVENT_TX_HALF,
    COMMUNICATION_EVENT_TX_CPLT
} communication_event_t;

void communication_init();
void communication_transmit(const void *data, const uint32_t size);
bool communication_receive(uint8_t *byte);
void communication_event(const communication_event_t event);

#endif
