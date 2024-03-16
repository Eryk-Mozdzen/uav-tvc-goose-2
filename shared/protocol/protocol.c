#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "protocol.h"

static const uint16_t crc16_lookup[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

static uint16_t crc16_calculate(const uint8_t *buffer, uint16_t size) {
    uint16_t crc = 0;

	while(size--) {
        crc = crc16_lookup[(crc^(*buffer++)) & 0xFF]^(crc>>8);
    }

    return crc;
}

static uint16_t cobs_encode(uint8_t *buffer, uint16_t size) {
    memmove(buffer + 1, buffer, size);
    size++;

    uint8_t *byte = buffer + 1;
    uint8_t *overhead = buffer;
    uint8_t counter = 1;

    for(uint16_t i=0; i<size-1; i++) {
        if(counter==0xFF) {
            *overhead = counter;
            overhead = byte;
            counter = 1;
            memmove(byte + 1, byte, size-i-1);
            size++;
            byte++;
            continue;
        }

        if(*byte) {
            counter++;
        } else {
            *overhead = counter;
            overhead = byte;
            counter = 1;
        }

        byte++;
    }

    *overhead = counter;

    buffer[size] = 0;
    size++;

    return size;
}

static uint16_t cobs_decode(uint8_t *buffer) {
    uint8_t overhead = buffer[0];
    uint8_t counter = 1;
    uint8_t *r = buffer + 1;
    uint8_t *w = buffer;

    while(*r) {
        if(counter==overhead) {
            overhead = *r;
            r++;
            if(counter!=0xFF) {
                *w = 0;
                w++;
            }
            counter = 1;
            continue;
        }

        *w = *r;
        w++;
        r++;
        counter++;
    }

    return (uint16_t)(w - buffer);
}

uint16_t protocol_encode(void *dest, const protocol_message_t *message) {
    uint8_t *d = (uint8_t *)dest;

    memcpy(&d[5], message->payload, message->size);

    d[4] = (message->size & 0xFF00)>>8;
    d[3] = (message->size & 0x00FF)>>0;
    d[2] = message->id;

    const uint16_t crc = crc16_calculate(&d[2], message->size + 3);

    d[1] = (crc & 0xFF00)>>8;
    d[0] = (crc & 0x00FF)>>0;

    return cobs_encode(dest, message->size + 5);
}

bool protocol_decode(protocol_decoder_t *decoder, const uint8_t byte, protocol_message_t *message) {
    decoder->buffer[decoder->counter] = byte;
    decoder->counter++;
    decoder->counter %=decoder->size;

    if(byte!=0) {
        return false;
    }

    if(decoder->counter<(5+1)) {
        decoder->counter = 0;
		return false;
	}

    const uint16_t length = cobs_decode(decoder->buffer);

    const uint16_t payload_size_p = (((uint16_t)decoder->buffer[4])<<8) | decoder->buffer[3];
	const uint16_t payload_size_r = length - 5;

    if(payload_size_p!=payload_size_r) {
        decoder->counter = 0;
    	return false;
    }

    message->size = payload_size_p;

	const uint16_t crc_p = (((uint16_t)decoder->buffer[1])<<8) | decoder->buffer[0];
	const uint16_t crc_r = crc16_calculate(decoder->buffer + 2, message->size + 3);

	if(crc_p!=crc_r) {
        decoder->counter = 0;
		return false;
	}

    message->id = decoder->buffer[2];

	memcpy(message->payload, decoder->buffer + 5, message->size);

    decoder->counter = 0;

	return true;
}
