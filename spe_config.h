#ifndef SPE_CONFIG_H
#define SPE_CONFIG_H

#include <stdint.h>

#define CONFIG_MAGIC 0xDEADBEEF

typedef struct {
    uint32_t magic;
    uint32_t counter;
    float threshold;
} ConfigData;

#endif // SPE_CONFIG_H
