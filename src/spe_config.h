#ifndef SPE_CONFIG_H
#define SPE_CONFIG_H
#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include <string.h>

#define CONFIG_MAGIC 0xDEADBEEF
#define CONFIG_VERSION 1

//Device configuration data structure stored in flash
#define CONFIG_SD 0
#define CONFIG_MD 1

typedef struct {
    uint32_t magic;       // Validates presence
    uint32_t version;     // Tracks structure version
    uint32_t mode;        // MD or SD mode
    uint32_t sd_num;        // SD number (for SD mode)
    float threshold;      // Another example
    uint32_t crc32;       // CRC of all fields above

} config_data_t;

extern config_data_t config_data;

int save_config(config_data_t *config);
int load_config(config_data_t *config);
uint32_t calculate_crc32(const void *data, size_t length);

#endif // SPE_CONFIG_H
