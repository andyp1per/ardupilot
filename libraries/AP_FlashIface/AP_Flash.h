/*
   AP_Logger logging - block oriented variant
 */
#pragma once

#include "AP_Flash_Backend.h"

#if HAL_FLASH_ENABLED

#define BLOCK_LOG_VALIDATE 0

class AP_Flash : {
public:
    AP_Flash();

    virtual void init(void) override = 0;
    virtual bool is_healthy(void) const override = 0;

    // get the current sector from the current page
    uint32_t get_sector(uint32_t current_page) const {
        return ((current_page - 1) / pages_per_sector);
    }

    // get the current block from the current page
    uint32_t get_block(uint32_t current_page) const {
        return ((current_page - 1) / pages_per_block);
    }

    /*
      functions implemented by the board specific backends
     */
    virtual void write_page(uint32_t PageAdr) = 0;
    virtual void read_page(uint32_t PageAdr) = 0;
    virtual void erase_sector(uint32_t SectorAdr) = 0;
    virtual void erase_4k_sector(uint32_t SectorAdr) = 0;
    virtual bool read_flash_config() = 0;
    virtual void wait_ready() = 0;
    virtual bool is_busy() = 0;

    void    flash_test(void);

    uint32_t get_page_size() const { return page_size; }
    uint16_t get_pages_per_block() const { return pages_per_block; }
    uint16_t get_pages_per_sector() const { return pages_per_sector; }
    uint32_t get_num_pages() const { return num_pages; }

protected:
    // number of bytes in a page
    uint32_t page_size;
    // number of pages in a (generally 64k) block
    uint16_t pages_per_block;
    // number of pages in a (generally 4k) sector
    uint16_t pages_per_sector;
    // number of pages on the chip
    uint32_t num_pages;

};

#endif  // HAL_LOGGING_BLOCK_ENABLED
