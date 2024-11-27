/*
  block based logging, for boards with flash logging
 */

#include "AP_Flash_config.h"

#if HAL_FLASH_ENABLED

#include "AP_Flash.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

const extern AP_HAL::HAL& hal;

AP_Flash::AP_Flash()
{
}

void AP_Flash::flash_test()
{
    const uint32_t pages_to_check = 128;
    for (uint32_t i=1; i<=pages_to_check; i++) {
        if ((i-1) % pages_per_block == 0) {
            printf("Block erase %u\n", get_block(i));
            erase_sector(get_block(i));
        }
        uint8_t buffer[page_size];
        memset(buffer, uint8_t(i), page_size);
        if (i<5) {
            printf("Flash fill 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash fill pages 5-%u\n", pages_to_check);
        }
        write_page(i, buffer);
    }
    for (uint32_t i=1; i<=pages_to_check; i++) {
        if (i<5) {
            printf("Flash check 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash check pages 5-%u\n", pages_to_check);
        }
        uint8_t buffer[page_size];
        read_page(i, buffer);
        uint32_t bad_bytes = 0;
        uint32_t first_bad_byte = 0;
        for (uint32_t j=0; j<page_size; j++) {
            if (buffer[j] != uint8_t(i)) {
                bad_bytes++;
                if (bad_bytes == 1) {
                    first_bad_byte = j;
                }
            }
        }
        if (bad_bytes > 0) {
            printf("Test failed: page %u, %u of %u bad bytes, first=0x%x\n",
                i, bad_bytes, page_size, buffer[first_bad_byte]);
        }
    }
}

#endif // HAL_LOGGING_BLOCK_ENABLED
