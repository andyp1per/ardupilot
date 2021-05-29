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
    Implements Common Flash Interface Driver based on
    Open Standard Published by JEDEC
*/

#include "AP_FlashIface_Abstract.h"


class AP_FlashIface_JEDEC : public AP_FlashIface
{
public:
    bool init() override;

    // Read methods
    bool read(uint32_t offset, uint8_t *data, uint32_t size) override;


    // Program methods
    uint32_t get_page_size() override
    {
        return _desc.page_size;
    }
    uint32_t get_page_count() override
    {
        return _desc.page_count;
    }
    bool start_program_page(uint32_t page, uint8_t *data, uint32_t &delay_us, uint32_t &timeout_us) override;
    bool start_program_offset(uint32_t offset, uint8_t* data, uint32_t size, uint32_t &programming,
                              uint32_t &delay_us, uint32_t &timeout_us) override;

    // Erase Methods
    bool start_mass_erase(uint32_t &delay_ms, uint32_t &timeout_ms) override;
    uint32_t get_sector_size() override
    {
        return _desc.sector_size;
    }
    uint32_t get_sector_count() override
    {
        return _desc.sector_count;
    }
    uint32_t min_erase_size() override
    {
        return _desc.min_erase_size;
    }
    bool start_sector_erase(uint32_t sector, uint32_t &delay_ms, uint32_t &timeout_ms) override;
    bool start_erase_offset(uint32_t offset, uint32_t size, uint32_t &erasing,
                            uint32_t &delay_ms, uint32_t &timeout_ms) override;
    bool verify_erase(uint32_t sector) override;

    bool is_device_busy() override;

protected:
    void reset_device();

    bool detect_device();

    bool configure_device();

    bool write_enable(bool quad_mode);
    bool write_disable(bool quad_mode);

    bool modify_reg(uint8_t read_ins, uint8_t write_ins,
                    uint8_t mask, uint8_t val, bool quad_mode);

    bool read_reg(uint8_t read_ins, uint8_t &read_val, bool quad_mode);

    bool write_reg(uint8_t read_ins, uint8_t write_val, bool quad_mode);

    bool send_cmd(uint8_t ins, bool quad_mode);

    bool _quad_spi_mode;

private:
    AP_HAL::OwnPtr<AP_HAL::QSPIDevice> _dev;

    // Device description extracted from SFDP
    struct device_desc {
        uint16_t param_rev;
        uint8_t param_table_len;
        uint32_t param_table_pointer;
        uint32_t flash_size;
        uint32_t page_size;
        uint32_t page_count;
        uint32_t sector_size;
        uint32_t sector_count;
        uint32_t min_erase_size;
        struct {
            uint8_t ins;
            uint32_t size;
            uint32_t delay_ms;
            uint32_t timeout_ms;
        } erase_type[4];
        uint32_t mass_erase_delay_ms;
        uint32_t mass_erase_timeout_ms;
        uint8_t write_enable_ins;
        uint32_t page_prog_delay_us;
        uint32_t page_prog_timeout_us;
        uint8_t fast_read_ins;
        uint8_t fast_read_dummy_cycles;
        uint8_t quad_mode_ins;
        bool quad_mode_rmw_seq;
        uint8_t status_read_ins;
        bool legacy_status_polling;
    } _desc;

    uint8_t _dev_list_idx;
    bool initialised;
    bool write_enable_called;
};

