#include "Util.h"

#if defined(RP2350)

#include <string.h>

void Pico::Util::_read_unique_id(uint8_t buf[12])
{
    // RP2350: read unique ID from OTP. Rows 0-3 = CHIPID (64-bit public
    // ID), rows 4-5 = RANDID[0:1] (32 more bits). OTP_DATA ECC-mapped
    // base 0x40130000U; each row is a 4-byte word with 16-bit data in
    // bits[15:0].
    const uint32_t otp_base = 0x40130000U;
    uint16_t tmp[6];
    for (uint32_t i = 0; i < 6U; i++) {
        tmp[i] = (uint16_t)(*(volatile const uint32_t *)(otp_base + i * 4U));
    }
    memcpy(buf, tmp, 12);
}

#endif // RP2350
