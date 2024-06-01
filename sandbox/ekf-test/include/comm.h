#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#define COMM_PACKET 16

typedef enum {
    COMM_EVENT_RX_HALF,
    COMM_EVENT_RX_CPLT,
    COMM_EVENT_TX_CPLT,
    COMM_EVENT_ERROR
} comm_event_t;

typedef struct {
    volatile uint8_t buffer[1024];
    volatile uint32_t read;
    volatile uint32_t write;
} comm_cb_t;

typedef struct {
    UART_HandleTypeDef *huart;
    comm_cb_t tx;
    comm_cb_t rx;
    uint8_t input[2*COMM_PACKET];
    uint8_t output[COMM_PACKET];
    volatile bool transmit_in_progress;
} comm_instance_t;

void comm_init(comm_instance_t *instance);
void comm_transmit(comm_instance_t *instance, const void *data, const uint32_t size);
bool comm_receive(comm_instance_t *instance, uint8_t *byte);
void comm_event(comm_instance_t *instance, const comm_event_t event);

#endif
