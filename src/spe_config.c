#include "spe_config.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FLASH_STORAGE_OFFSET (2 * 1024 * 1024 - FLASH_SECTOR_SIZE)
#ifndef XIP_BASE
#define XIP_BASE 0x10000000  // (only if you really need a local value)
#endif

int save_config(config_data_t *config) {
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    config->crc32 = calculate_crc32(config, sizeof(config_data_t) - sizeof(uint32_t));

    uint8_t buffer[FLASH_PAGE_SIZE] = {0};
    memcpy(buffer, config, sizeof(config_data_t));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STORAGE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STORAGE_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

 int load_config(config_data_t *config) {
    const config_data_t *stored = (const config_data_t *)(XIP_BASE + FLASH_STORAGE_OFFSET);
    uint32_t expected_crc = calculate_crc32(stored, sizeof(config_data_t) - sizeof(uint32_t));

    if (stored->magic == CONFIG_MAGIC &&
        stored->version == CONFIG_VERSION &&
        stored->crc32 == expected_crc) {
        config->mode = stored->mode;
        return 0;
    } else {
        return 1;
    }
}


uint32_t calculate_crc32(const void *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }

    return ~crc;
}
