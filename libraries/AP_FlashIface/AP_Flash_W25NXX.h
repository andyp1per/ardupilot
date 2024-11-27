/*
  logging for block based dataflash devices on SPI
 */
#pragma once

#include "AP_Flash_config.h"

#if HAL_FLASH_W25NXX_ENABLED

#include <AP_HAL/AP_HAL.h>

class AP_Flash_W25NXX : public AP_Flash {
public:
    AP_Flash_W25NXX() : AP_Flash() {}
    static AP_Flash  *probe() {
        return NEW_NOTHROW AP_Flash_W25NXX();
    }
    void              init(void) override;
    bool              is_healthy() const override { return !flash_died && df_NumPages > 0; }
    void              write_page(uint32_t PageAdr, uint8_t* buffer) override;
    void              read_page(uint32_t PageAdr, uint8_t* buffer) override;
    void              erase_sector(uint32_t SectorAdr) override;
    void              erase_4k_sector(uint32_t SectorAdr) override;
    void              wait_ready() override;
    bool              is_busy() override;
    bool              read_flash_config(void) override;

private:
    void              send_command_addr(uint8_t cmd, uint32_t address);
    uint8_t           read_status_register(uint8_t bits);
    void              write_status_register(uint8_t reg, uint8_t bits);
    void              write_enable();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    AP_HAL::Semaphore *dev_sem;

    uint32_t flash_blockNum;

    bool flash_died;
    bool read_cache_valid;
};

#endif // HAL_FLASH_W25NXX_ENABLED
