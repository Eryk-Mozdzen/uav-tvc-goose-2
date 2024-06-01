#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>
#include <stdbool.h>

#define NMEA_MAX_FIELD_NUM     24
#define NMEA_MAX_FIELD_SIZE    16

struct NMEA_Context {
    char tail[4];
    uint8_t state;
    uint8_t checksum;
    uint8_t field_counter;
    uint8_t char_counter;
};

typedef struct {
    uint8_t argc;
    char argv[NMEA_MAX_FIELD_NUM][NMEA_MAX_FIELD_SIZE];
    struct NMEA_Context context;
} NMEA_Message_t;

bool NMEA_Consume(NMEA_Message_t *message, const char byte);

#endif
