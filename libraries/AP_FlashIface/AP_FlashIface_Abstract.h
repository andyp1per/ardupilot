/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */
/*
    Implements Common frontend methods for Flash Interface Driver
*/
#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_FlashIface
{

public:
    virtual bool init() = 0;

    virtual bool read(uint32_t offset, uint8_t* data, uint32_t size) = 0;

    virtual uint32_t get_page_size() = 0;
    virtual uint32_t get_page_count() = 0;
    virtual bool start_program_page(uint32_t page, uint8_t *data, uint32_t &delay_ms, uint32_t &timeout_ms) = 0;
    virtual bool start_program_offset(uint32_t offset, uint8_t* data, uint32_t size, uint32_t &programming,
                                      uint32_t &delay_ms, uint32_t &timeout_ms)
    {
        return false;
    }

    virtual uint32_t get_sector_size() = 0;
    virtual uint32_t get_sector_count() = 0;
    virtual bool start_mass_erase(uint32_t &delay_ms, uint32_t &timeout_ms) = 0;
    virtual bool start_sector_erase(uint32_t sector, uint32_t &delay_ms, uint32_t &timeout_ms) = 0;
    virtual bool start_erase_offset(uint32_t offset, uint32_t size, uint32_t &erasing,
                                    uint32_t &delay_ms, uint32_t &timeout_ms)
    {
        return false;
    }
    virtual bool is_device_busy() = 0;
    virtual bool verify_erase(uint32_t sector) = 0;
    virtual uint32_t min_erase_size() = 0;

};
