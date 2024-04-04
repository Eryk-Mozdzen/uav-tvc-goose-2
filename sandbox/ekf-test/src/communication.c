#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "communication.h"

#define PACKET 8

extern UART_HandleTypeDef huart2;

typedef struct {
    volatile uint8_t buffer[1024];
    volatile uint32_t read;
    volatile uint32_t write;
} cb_t;

static cb_t tx;
static cb_t rx;

static uint8_t input[2*PACKET];
static uint8_t output[2*PACKET];

static inline void cb_write(cb_t *cb, const uint8_t byte) {
    cb->buffer[cb->write] = byte;
    cb->write++;
    cb->write %=sizeof(cb->buffer);
}

static inline bool cb_read(cb_t *cb, uint8_t *byte) {
    if(cb->read==cb->write) {
        return false;
    }
    *byte = cb->buffer[cb->read];
    cb->read++;
    cb->read %=sizeof(cb->buffer);
    return true;
}

void communication_init() {
    HAL_UART_Receive_DMA(&huart2, input, 2*PACKET);
    HAL_UART_Transmit_DMA(&huart2, output, 2*PACKET);
}

void communication_transmit(const void *data, const uint32_t size) {
    __disable_irq();
    for(uint32_t i=0; i<size; i++) {
        cb_write(&tx, ((uint8_t *)data)[i]);
    }
    __enable_irq();
}

bool communication_receive(uint8_t *byte) {
    __disable_irq();
    const bool result = cb_read(&rx, byte);
    __enable_irq();
    return result;
}

void communication_event(const communication_event_t event) {
    switch(event) {
        case COMMUNICATION_EVENT_RX_HALF: {
            uint8_t i = 0;
            while(i<PACKET) {
                cb_write(&rx, input[i]);
                i++;
            }
        } break;
        case COMMUNICATION_EVENT_RX_CPLT: {
            uint8_t i = 0;
            while(i<PACKET) {
                cb_write(&rx, input[i + PACKET]);
                i++;
            }
        } break;
        case COMMUNICATION_EVENT_TX_HALF: {
            uint8_t i = 0;
            while(i<PACKET && cb_read(&tx, &output[i])) {
                i++;
            }
            while(i<PACKET) {
                output[i] = 0;
                i++;
            }
        } break;
        case COMMUNICATION_EVENT_TX_CPLT: {
            uint8_t i = 0;
            while(i<PACKET && cb_read(&tx, &output[i + PACKET])) {
                i++;
            }
            while(i<PACKET) {
                output[i + PACKET] = 0;
                i++;
            }
            HAL_UART_Transmit_DMA(&huart2, output, 2*PACKET);
        } break;
    }
}
