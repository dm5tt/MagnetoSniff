#ifndef SENDER_H
#define SENDER_H

#include <stdint.h>
#include <stddef.h>

#define SENDER_SAMPLES_PER_BUFFER 400

typedef struct __attribute__((packed))
{
    uint64_t timestamp; // 8 bytes
    float x;            // 4 bytes
    float y;            // 4 bytes
    float z;            // 4 bytes
    float temperature;  // 4 bytes
} sender_mag_sample_t;

void sender_init(void);
void sender_send_buffer(sender_mag_sample_t *buffer, size_t length);

#endif // SENDER_H
