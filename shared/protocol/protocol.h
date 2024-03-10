#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

uint16_t protocol_encode(const uint8_t id, const void *payload, const uint16_t payload_size, void *dest);
bool protocol_decode(const uint8_t byte, uint8_t *id, void *payload, uint16_t *payload_size);

#ifdef __cplusplus
    }
#endif

#endif
