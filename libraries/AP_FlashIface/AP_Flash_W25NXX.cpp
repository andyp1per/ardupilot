/*
  logging to a DataFlash block based storage device on SPI
*/


#include <AP_HAL/AP_HAL.h>

#include "AP_Flash_W25NXX.h"

#if HAL_FLASH_W25NXX_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_PAGE_DATA_READ         0x13
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_PROGRAM_EXECUTE        0x10

#define JEDEC_DEVICE_RESET           0xFF
#define JEDEC_BLOCK_ERASE            0xD8 // 128K erase

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02

#define W25NXX_STATUS_REG           0xC0
#define W25NXX_PROT_REG             0xA0
#define W25NXX_CONF_REG             0xB0
#define W25NXX_STATUS_EFAIL         0x04
#define W25NXX_STATUS_PFAIL         0x08

#define W25NXX_PROT_SRP1_ENABLE          (1 << 0)
#define W25NXX_PROT_WP_E_ENABLE          (1 << 1)
#define W25NXX_PROT_TB_ENABLE            (1 << 2)
#define W25NXX_PROT_PB0_ENABLE           (1 << 3)
#define W25NXX_PROT_PB1_ENABLE           (1 << 4)
#define W25NXX_PROT_PB2_ENABLE           (1 << 5)
#define W25NXX_PROT_PB3_ENABLE           (1 << 6)
#define W25NXX_PROT_SRP2_ENABLE          (1 << 7)

#define W25NXX_CONFIG_ECC_ENABLE         (1 << 4)
#define W25NXX_CONFIG_BUFFER_READ_MODE   (1 << 3)

#define W25NXX_TIMEOUT_PAGE_READ_US        60   // tREmax = 60us (ECC enabled)
#define W25NXX_TIMEOUT_PAGE_PROGRAM_US     700  // tPPmax = 700us
#define W25NXX_TIMEOUT_BLOCK_ERASE_MS      10   // tBEmax = 10ms
#define W25NXX_TIMEOUT_RESET_MS            500  // tRSTmax = 500ms

#define W25N01G_NUM_BLOCKS                  1024
#define W25N02K_NUM_BLOCKS                  2048

#define JEDEC_ID_WINBOND_W25N01GV      0xEFAA21
#define JEDEC_ID_WINBOND_W25N02KV      0xEFAA22

void AP_Flash_W25NXX::init()
{
    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        AP_HAL::panic("PANIC: AP_Logger W25NXX device not found");
        return;
    }

    dev_sem = dev->get_semaphore();

    if (get_num_sectors() == 0) {
        flash_died = true;
        return;
    }

    flash_died = false;

    // reset the device
    wait_ready();
    {
        WITH_SEMAPHORE(dev_sem);
        uint8_t b = JEDEC_DEVICE_RESET;
        dev->transfer(&b, 1, nullptr, 0);
    }
    hal.scheduler->delay(W25NXX_TIMEOUT_RESET_MS);

    // disable write protection
    write_status_register(W25NXX_PROT_REG, 0);
    // enable ECC and buffer mode
    write_status_register(W25NXX_CONF_REG, W25NXX_CONFIG_ECC_ENABLE|W25NXX_CONFIG_BUFFER_READ_MODE);

    printf("W25NXX status: SR-1=0x%x, SR-2=0x%x, SR-3=0x%x\n",
        read_status_register(W25NXX_PROT_REG),
        read_status_register(W25NXX_CONF_REG),
        read_status_register(W25NXX_STATUS_REG));
}

/*
  wait for busy flag to be cleared
 */
void AP_Flash_W25NXX::wait_ready()
{
    if (flash_died) {
        return;
    }

    uint32_t t = AP_HAL::millis();
    while (is_busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            printf("W25NXX: flash_died\n");
            flash_died = true;
            break;
        }
    }
}

bool AP_Flash_W25NXX::read_flash_config(void)
{
    wait_ready();

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4]; // buffer not yet allocated
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[1] << 16 | buf[2] << 8 | buf[3];

    switch (id) {
    case JEDEC_ID_WINBOND_W25N01GV:
        page_size = 2048;
        pages_per_block = 64;
        pages_per_sector = 64; // make sectors equivalent to block
        flash_blockNum = W25N01G_NUM_BLOCKS;
        break;
    case JEDEC_ID_WINBOND_W25N02KV:
        page_size = 2048;
        pages_per_block = 64;
        pages_per_sector = 64; // make sectors equivalent to block
        flash_blockNum = W25N02K_NUM_BLOCKS;
        break;

    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    num_pages = flash_blockNum * pages_per_block;

    printf("SPI Flash 0x%08x found pages=%u\n", id, num_pages);
    return true;
}

// Read the status register bits
uint8_t AP_Flash_W25NXX::read_status_register(uint8_t bits)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[2] { JEDEC_READ_STATUS, bits };
    uint8_t status;
    dev->transfer(cmd, 2, &status, 1);
    return status;
}

void AP_Flash_W25NXX::write_status_register(uint8_t reg, uint8_t bits)
{
    wait_ready();
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[3] = {JEDEC_WRITE_STATUS, reg, bits};
    dev->transfer(cmd, 3, nullptr, 0);
}

bool AP_Flash_W25NXX::is_busy()
{
    uint8_t status = read_status_register(W25NXX_STATUS_REG);

    if ((status & W25NXX_STATUS_PFAIL) != 0) {
        printf("Program failure!\n");
    }
    if ((status & W25NXX_STATUS_EFAIL) != 0) {
        printf("Erase failure!\n");
    }

    return (status & JEDEC_STATUS_BUSY) != 0;
}

/*
  send a command with an address
*/
void AP_Flash_W25NXX::send_command_addr(uint8_t command, uint32_t PageAdr)
{
    uint8_t cmd[4];
    cmd[0] = command;
    cmd[1] = (PageAdr >>  16) & 0xff;
    cmd[2] = (PageAdr >>  8) & 0xff;
    cmd[3] = (PageAdr >>  0) & 0xff;

    dev->transfer(cmd, 4, nullptr, 0);
}

bool AP_Flash_W25NXX::read_page(uint32_t pageNum, uint8_t* buffer)
{
    if (pageNum == 0 || pageNum > num_pages+1) {
        return false;
    }

    wait_ready();

    uint32_t PageAdr = (pageNum-1);

    {
        WITH_SEMAPHORE(dev_sem);
        // read page into internal buffer
        send_command_addr(JEDEC_PAGE_DATA_READ, PageAdr);
    }

    // read from internal buffer into our buffer
    wait_ready();
    {
        WITH_SEMAPHORE(dev_sem);
        dev->set_chip_select(true);
        uint8_t cmd[4];
        cmd[0] = JEDEC_READ_DATA;
        cmd[1] = (0 >>  8) & 0xff; // column address zero
        cmd[2] = (0 >>  0) & 0xff; // column address zero
        cmd[3] = 0; // dummy
        dev->transfer(cmd, 4, nullptr, 0);
        dev->transfer(nullptr, 0, buffer, page_size);
        dev->set_chip_select(false);
    }
}

bool AP_Flash_W25NXX::write_page(uint32_t pageNum, uint8_t* buffer)
{
    if (pageNum == 0 || pageNum > num_pages+1) {
        return false;
    }

    write_enable();

    uint32_t PageAdr = (pageNum-1);
    {
        WITH_SEMAPHORE(dev_sem);

        // write our buffer into internal buffer
        dev->set_chip_select(true);

        uint8_t cmd[3];
        cmd[0] = JEDEC_PAGE_WRITE;
        cmd[1] = (0 >>  8) & 0xff; // column address zero
        cmd[2] = (0 >>  0) & 0xff; // column address zero

        dev->transfer(cmd, 3, nullptr, 0);
        dev->transfer(buffer, page_size, nullptr, 0);
        dev->set_chip_select(false);
    }

    // write from internal buffer into page
    {
        WITH_SEMAPHORE(dev_sem);
        send_command_addr(JEDEC_PROGRAM_EXECUTE, PageAdr);
    }
}

/*
  erase one sector (sizes varies with hw)
*/
void AP_Flash_W25NXX::erase_sector(uint32_t blockNum)
{
    write_enable();
    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = blockNum  * pages_per_block;
    send_command_addr(JEDEC_BLOCK_ERASE, PageAdr);
}

/*
  erase one 4k sector
*/
void AP_Flash_W25NXX::erase_4k_sector(uint32_t sectorNum)
{
    erase_sector(sectorNum);
}

void AP_Flash_W25NXX::write_enable(void)
{
    wait_ready();
    WITH_SEMAPHORE(dev_sem);
    uint8_t b = JEDEC_WRITE_ENABLE;
    dev->transfer(&b, 1, nullptr, 0);
}

#endif // HAL_LOGGING_FLASH_W25NXXHAL_FLASH_W25NXX_ENABLED_ENABLED
