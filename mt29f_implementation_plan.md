# MT29F1G01ABAFDWB LittleFS Support Implementation Plan

## Overview

Add support for the Micron MT29F1G01ABAFDWB-IT:F 1Gbit SPI NAND flash chip to ArduPilot's LittleFS filesystem driver.

## Chip Specifications

| Parameter | Value |
|-----------|-------|
| Manufacturer | Micron (0x2C) |
| Device ID | 0x14 |
| JEDEC ID (3-byte) | 0x2C142C |
| Capacity | 1Gbit (128MiB) |
| Page Size | 2048 bytes |
| Block Size | 128KB (131072 bytes) |
| Blocks | 1024 |
| Pages per Block | 64 |

## Technical Analysis

### Command Set Comparison

| Command | MT29F | W25NXX | Notes |
|---------|-------|--------|-------|
| Reset | 0xFF | 0xFF | Same |
| Read ID | 0x9F | 0x9F | Same |
| **Set Feature** | **0x1F** | 0x01 | **Different** |
| **Get Feature** | **0x0F** | 0x05 | **Different** |
| Page Read to Cache | 0x13 | 0x13 | Same |
| Read from Cache | 0x03 | 0x03 | Same |
| Write Enable | 0x06 | 0x06 | Same |
| Program Load | 0x02 | 0x02 | Same |
| Program Execute | 0x10 | 0x10 | Same |
| Block Erase | 0xD8 | 0xD8 | Same |

### Register Addresses (Same for both)

| Register | Address |
|----------|---------|
| Block Lock/Protection | 0xA0 |
| Configuration | 0xB0 |
| Status | 0xC0 |

### Key Difference

The only significant difference is the Set/Get Feature commands:
- W25NXX uses JEDEC NOR-style: 0x01 (write) / 0x05 (read)
- MT29F uses ONFI-style: 0x1F (set) / 0x0F (get)

## Implementation Approach

Since the chips are very similar (same page/block sizes, same register layout, same read/write/erase protocol), we will extend the existing W25NXX code path rather than creating a separate driver.

### Changes Required

#### 1. Add JEDEC ID constant (~line 620)

```cpp
#define JEDEC_ID_MICRON_MT29F1G         0x2C142C  // 128MiB (1Gb), 1024 blocks
```

#### 2. Add MT29F-specific command definitions (~line 578)

```cpp
#define MT29F_SET_FEATURE            0x1F
#define MT29F_GET_FEATURE            0x0F
```

#### 3. Add chip type detection flag (class member)

```cpp
bool is_mt29f;  // true for MT29F, false for W25NXX
```

#### 4. Modify `find_block_size_and_count()` (~line 759)

Add case for MT29F1G:
```cpp
case JEDEC_ID_MICRON_MT29F1G:
    block_count = 1024;   /* 128MiB */
    is_mt29f = true;
    break;
```

#### 5. Modify `is_busy()` (~line 641)

Use correct Get Feature command based on chip type:
```cpp
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_W25NXX
    uint8_t cmd[2] { is_mt29f ? MT29F_GET_FEATURE : JEDEC_READ_STATUS, W25NXX_STATUS_REG };
    dev->transfer(cmd, 2, &status, 1);
    return (status & (JEDEC_STATUS_BUSY | W25NXX_STATUS_PFAIL | W25NXX_STATUS_EFAIL)) != 0;
#else
```

#### 6. Modify `write_status_register()` (~line 703)

Use correct Set Feature command based on chip type:
```cpp
void AP_Filesystem_FlashMemory_LittleFS::write_status_register(uint8_t reg, uint8_t bits)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[3] = { is_mt29f ? MT29F_SET_FEATURE : JEDEC_WRITE_STATUS, reg, bits };
    dev->transfer(cmd, 3, nullptr, 0);
}
```

#### 7. Update header file

Add `is_mt29f` member variable to the class.

## Files to Modify

1. `libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp`
   - Add JEDEC ID constant
   - Add MT29F command definitions
   - Add JEDEC ID case in `find_block_size_and_count()`
   - Modify `is_busy()` to use correct Get Feature command
   - Modify `write_status_register()` to use correct Set Feature command

2. `libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.h`
   - Add `is_mt29f` member variable

## Testing

1. Build with `./waf configure --board <board-with-mt29f> --enable-littlefs`
2. Verify JEDEC ID detection in boot messages
3. Test filesystem operations (format, read, write, erase)

## hwdef.dat Configuration

Boards using MT29F will use the existing configuration:
```
SPIDEV dataflash SPI<n> DEVID<m> FLASH_CS MODE3 <clock>*MHZ <clock>*MHZ
DATAFLASH littlefs:w25nxx
```

No hwdef changes required - the chip is detected automatically by JEDEC ID.

## References

- Betaflight PR #14828: https://github.com/betaflight/betaflight/pull/14828
- Micron MT29F1G01 Datasheet (registration required)
- GitHub flash_management project: https://github.com/aloebs29/flash_management
