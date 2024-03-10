#ifndef PROTOCOL_DATA_H
#define PROTOCOL_DATA_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdint.h>

typedef enum {
    PROTOCOL_ID_LOG_ERROR,
    PROTOCOL_ID_LOG_WARNING,
    PROTOCOL_ID_LOG_INFO,
    PROTOCOL_ID_LOG_DEBUG
} protocol_id_t;

typedef struct {
    float pre[3];
    float post[3];
} protocol_sensor3d_t;

#ifdef __cplusplus
    }
#endif

#endif
