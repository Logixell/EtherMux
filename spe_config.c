#include "spe_config.h"
#include "hardware/flash.h"
#include "pico/stdlib.h"
#include <string.h>

#define FLASH_STORAGE_OFFSET (2 * 1024 * 1024 - FLASH_SECTOR_SIZE)
#define XIP_BASE 0x10000000

void save_config(ConfigData *config) {
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    config->crc32 = calculate_crc32(config, sizeof(ConfigData) - sizeof(uint32_t));

    uint8_t buffer[FLASH_PAGE_SIZE] = {0};
    memcpy(buffer, config, sizeof(ConfigData));

    flash_range_erase(FLASH_STORAGE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STORAGE_OFFSET, buffer, FLASH_PAGE_SIZE);
}

ConfigData load_config() {
    const ConfigData *stored = (const ConfigData *)(XIP_BASE + FLASH_STORAGE_OFFSET);
    uint32_t expected_crc = calculate_crc32(stored, sizeof(ConfigData) - sizeof(uint32_t));

    if (stored->magic == CONFIG_MAGIC &&
        stored->version == CONFIG_VERSION &&
        stored->crc32 == expected_crc) {
        return *stored;
    } else {
        return (ConfigData){.magic = CONFIG_MAGIC, .version = CONFIG_VERSION, .mode = 0, .threshold = 1.0f};
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
