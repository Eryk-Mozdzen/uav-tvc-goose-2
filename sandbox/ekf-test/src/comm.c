#include <stdint.h>
#include <stdbool.h>

#include "comm.h"

static inline void cb_write(comm_cb_t *cb, const uint8_t byte) {
    cb->buffer[cb->write] = byte;
    cb->write++;
    cb->write %=sizeof(cb->buffer);
}

static inline bool cb_read(comm_cb_t *cb, uint8_t *byte) {
    if(cb->read==cb->write) {
        return false;
    }
    *byte = cb->buffer[cb->read];
    cb->read++;
    cb->read %=sizeof(cb->buffer);
    return true;
}

void comm_init(comm_instance_t *instance) {
    HAL_UART_Receive_DMA(instance->huart, instance->input, 2*COMM_PACKET);
}

void comm_transmit(comm_instance_t *instance, const void *data, const uint32_t size) {
    __disable_irq();
    for(uint32_t i=0; i<size; i++) {
        cb_write(&instance->tx, ((uint8_t *)data)[i]);
    }
    __enable_irq();

    if(!instance->transmit_in_progress) {
        __disable_irq();
        instance->transmit_in_progress = true;
        uint8_t i = 0;
        while(i<COMM_PACKET && cb_read(&instance->tx, &instance->output[i])) {
            i++;
        }
        __enable_irq();
        HAL_UART_Transmit_DMA(instance->huart, instance->output, i);
    }
}

bool comm_receive(comm_instance_t *instance, uint8_t *byte) {
    __disable_irq();
    const bool result = cb_read(&instance->rx, byte);
    __enable_irq();
    return result;
}

void comm_event(comm_instance_t *instance, const comm_event_t event) {
    switch(event) {
        case COMM_EVENT_RX_HALF: {
            uint8_t i = 0;
            while(i<COMM_PACKET) {
                cb_write(&instance->rx, instance->input[i]);
                i++;
            }
        } break;
        case COMM_EVENT_RX_CPLT: {
            uint8_t i = 0;
            while(i<COMM_PACKET) {
                cb_write(&instance->rx, instance->input[i + COMM_PACKET]);
                i++;
            }
        } break;
        case COMM_EVENT_TX_CPLT: {
            uint8_t i = 0;
            while(i<COMM_PACKET && cb_read(&instance->tx, &instance->output[i])) {
                i++;
            }
            if(i>0) {
                HAL_UART_Transmit_DMA(instance->huart, instance->output, i);
            } else {
                instance->transmit_in_progress = false;
            }
        } break;
        case COMM_EVENT_ERROR: {
            HAL_UART_Receive_DMA(instance->huart, instance->input, 2*COMM_PACKET);
        } break;
    }
}
