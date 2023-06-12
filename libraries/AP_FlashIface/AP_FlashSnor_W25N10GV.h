/*
  logging for block based dataflash devices on SPI
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "hal_serial_nor.h"

class AP_FlashSnor_W25N01GV {
public:
    AP_FlashSnor_W25N01GV() {}

private:

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    AP_HAL::Semaphore *dev_sem;
};

extern flash_descriptor_t snor_descriptor;

extern "C" {
  void snor_device_init(SNORDriver *devp);
  flash_error_t snor_device_read(SNORDriver *devp, flash_offset_t offset,
                                 size_t n, uint8_t *rp);
  flash_error_t snor_device_program(SNORDriver *devp, flash_offset_t offset,
                                    size_t n, const uint8_t *pp);
  flash_error_t snor_device_start_erase_all(SNORDriver *devp);
  flash_error_t snor_device_start_erase_sector(SNORDriver *devp,
                                               flash_sector_t sector);
  flash_error_t snor_device_verify_erase(SNORDriver *devp,
                                         flash_sector_t sector);
  flash_error_t snor_device_query_erase(SNORDriver *devp, uint32_t *msec);
  flash_error_t snor_device_read_sfdp(SNORDriver *devp, flash_offset_t offset,
                                      size_t n, uint8_t *rp);
}
