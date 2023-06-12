/*
  ArduPilot filesystem interface for systems using the LittleFS filesystem in
  flash memory
*/
#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_LITTLEFS_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_FlashMemory_LittleFS.h"

#include "lfs.h"

#if 0
#define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#define ENSURE_MOUNTED() do { if (!mounted && !mount_filesystem()) { errno = EIO; return -1; }} while (0)
#define ENSURE_MOUNTED_NULL() do { if (!mounted && !mount_filesystem()) { errno = EIO; return nullptr; }} while (0)
#define LFS_CHECK(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return -1; }} while (0)
#define LFS_CHECK_NULL(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return nullptr; }} while (0)

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
);
static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
);
static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block);
static int flashmem_sync(const struct lfs_config *cfg);

static int errno_from_lfs_error(int lfs_error);
static int lfs_flags_from_flags(int flags);

const extern AP_HAL::HAL& hal;

int AP_Filesystem_FlashMemory_LittleFS::open(const char *pathname, int flags, bool allow_absolute_path)
{
    int fd, retval;
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);

    ENSURE_MOUNTED();

    fd = allocate_fd();
    if (fd < 0) {
        return -1;
    }

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    retval = lfs_file_open(&fs, fp, pathname, lfs_flags_from_flags(flags));
    if (retval < 0) {
        errno = errno_from_lfs_error(retval);
        free_fd(fd);
        return -1;
    }

    return fd;
}

int AP_Filesystem_FlashMemory_LittleFS::close(int fileno)
{
    lfs_file_t* fp;
    int retval;

    WITH_SEMAPHORE(fs_sem);

    fp = lfs_file_from_fd(fileno);
    if (fp == nullptr) {
        return -1;
    }

    retval = lfs_file_close(&fs, fp);
    if (retval < 0) {
        free_fd(fileno);   // ignore error code, we have something else to report
        errno = errno_from_lfs_error(retval);
        return -1;
    }

    if (free_fd(fileno) < 0) {
        return -1;
    }

    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::read(int fd, void *buf, uint32_t count)
{
    lfs_file_t* fp;
    lfs_ssize_t read;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    read = lfs_file_read(&fs, fp, buf, count);
    if (read < 0) {
        errno = errno_from_lfs_error(read);
        return -1;
    }

    return read;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::write(int fd, const void *buf, uint32_t count)
{
    lfs_file_t* fp;
    lfs_ssize_t written;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }
    
    written = lfs_file_write(&fs, fp, buf, count);
    if (written < 0) {
        errno = errno_from_lfs_error(written);
        return -1;
    }

    return written;
}

int AP_Filesystem_FlashMemory_LittleFS::fsync(int fd)
{
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    LFS_CHECK(lfs_file_sync(&fs, fp));
    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::lseek(int fd, int32_t position, int whence)
{
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    LFS_CHECK(lfs_file_seek(&fs, fp, position, whence));
    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::stat(const char *name, struct stat *buf)
{
    lfs_info info;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    
    LFS_CHECK(lfs_stat(&fs, name, &info));

    memset(buf, 0, sizeof(*buf));
    buf->st_mode = (info.type == LFS_TYPE_DIR ? S_IFREG : S_IFDIR) | 0666;
    buf->st_nlink = 1;
    buf->st_size = info.size;
    buf->st_blksize = fs_cfg.read_size;    
    buf->st_blocks = (info.size >> 9) + ((info.size & 0x1FF) > 0 ? 1 : 0);

    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::unlink(const char *pathname)
{
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_remove(&fs, pathname));
    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::mkdir(const char *pathname)
{
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_mkdir(&fs, pathname));
    return 0;
}

typedef struct {
    lfs_dir_t dir;
    struct dirent entry;
} lfs_dir_entry_pair;

void *AP_Filesystem_FlashMemory_LittleFS::opendir(const char *pathdir)
{
    int retval;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED_NULL();

    lfs_dir_entry_pair *result = new lfs_dir_entry_pair;
    if (!result) {
        errno = ENOMEM;
        return nullptr;
    }

    retval = lfs_dir_open(&fs, &result->dir, pathdir);
    if (retval < 0) {
        delete result;
        errno = errno_from_lfs_error(retval);
        return nullptr;
    }

    memset(&result->entry, 0, sizeof(result->entry));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    result->entry.d_reclen = sizeof(result->entry);
#endif

    return result;
}

struct dirent *AP_Filesystem_FlashMemory_LittleFS::readdir(void *ptr)
{
    WITH_SEMAPHORE(fs_sem);

    lfs_info info;
    lfs_dir_entry_pair *pair = static_cast<lfs_dir_entry_pair*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return nullptr;
    }

    if (!lfs_dir_read(&fs, &pair->dir, &info)) {
        /* no more entries */
        return nullptr;
    }

    memset(&pair->entry, 0, sizeof(pair->entry));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    pair->entry.d_ino = 0;
    pair->entry.d_seekoff++;
#endif

    strncpy(pair->entry.d_name, info.name, sizeof(pair->entry.d_name));
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    pair->entry.d_namlen = strlen(info.name);
#endif

    pair->entry.d_type = info.type == LFS_TYPE_DIR ? DT_DIR : DT_REG;

    return &pair->entry;
}

int AP_Filesystem_FlashMemory_LittleFS::closedir(void *ptr)
{
    WITH_SEMAPHORE(fs_sem);

    lfs_dir_entry_pair *pair = static_cast<lfs_dir_entry_pair*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return 0;
    }

    LFS_CHECK(lfs_dir_close(&fs, &pair->dir));

    delete pair;

    return 0;
}

int64_t AP_Filesystem_FlashMemory_LittleFS::disk_free(const char *path)
{
    lfs_ssize_t alloc_size;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    alloc_size = lfs_fs_size(&fs);
    if (alloc_size < 0) {
        errno = errno_from_lfs_error(alloc_size);
        return -1;
    }

    return disk_space(path) - alloc_size;
}

int64_t AP_Filesystem_FlashMemory_LittleFS::disk_space(const char *path)
{
    return fs_cfg.block_count * fs_cfg.block_size;
}

bool AP_Filesystem_FlashMemory_LittleFS::retry_mount(void)
{
    WITH_SEMAPHORE(fs_sem);

    if (!dead) {
        if (!mounted && !mount_filesystem()) {
            errno = EIO;
            return false;
        }

        return true;
    } else {
        return false;
    }
}

void AP_Filesystem_FlashMemory_LittleFS::unmount(void)
{
    WITH_SEMAPHORE(fs_sem);

    if (mounted && !dead) {
        if (lfs_unmount(&fs) >= 0) {
            mounted = false;
        }
    }
}

/* ************************************************************************* */
/* Private functions                                                         */
/* ************************************************************************* */

int AP_Filesystem_FlashMemory_LittleFS::allocate_fd()
{
    int fd;

    for (fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] == nullptr) {
            open_files[fd] = static_cast<lfs_file_t*>(calloc(1, sizeof(lfs_file_t)));
            if (open_files[fd] == nullptr) {
                errno = ENOMEM;
                return -1;
            }

            return fd;
        }
    }

    errno = ENFILE;
    return -1;
}

int AP_Filesystem_FlashMemory_LittleFS::free_fd(int fd)
{
    lfs_file_t* fp = lfs_file_from_fd(fd);
    if (!fp) {
        return -1;
    }

    free(fp);
    open_files[fd] = fp = nullptr;

    return 0;
}

void AP_Filesystem_FlashMemory_LittleFS::free_all_fds()
{
    int fd;

    for (fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] == nullptr) {
            free_fd(fd);
        }
    }
}

lfs_file_t* AP_Filesystem_FlashMemory_LittleFS::lfs_file_from_fd(int fd) const
{
    if (fd < 0 || fd >= MAX_OPEN_FILES || open_files[fd] == nullptr) {
        errno = EBADF;
        return nullptr;
    }

    return open_files[fd];
}

void AP_Filesystem_FlashMemory_LittleFS::mark_dead()
{
    if (!dead) {
        printf("FlashMemory_LittleFS: dead\n");
        free_all_fds();
        dead = true;
    }
}

/* ************************************************************************* */
/* Low-level flash memory access                                             */
/* ************************************************************************* */

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_PAGE_DATA_READ         0x13
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_PROGRAM_EXECUTE        0x10

#define JEDEC_DEVICE_RESET           0xFF

#define JEDEC_BULK_ERASE             0xC7
#define JEDEC_SECTOR4_ERASE          0x20 // 4k erase
#define JEDEC_BLOCK_ERASE            0xD8

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

/*
  flash device IDs taken from betaflight flash_m25p16.c

  Format is manufacturer, memory type, then capacity
*/
#define JEDEC_ID_UNKNOWN               0x000000
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20BA18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25X32        0xEF3016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q256       0xEF4019
#define JEDEC_ID_WINBOND_W25Q128_2     0xEF7018
#define JEDEC_ID_WINBOND_W25N01GV      0xEFAA21
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018

/* Hardware-specific constants */

#define W25N01G_PROT_REG             0xA0
#define W25N01G_CONF_REG             0xB0
#define W25N01G_STATUS_REG           0xC0

#define W25N01G_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE   (1 << 3)

#define W25N01G_TIMEOUT_PAGE_READ_US        60   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_US     700  // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS      10   // tBEmax = 10ms
#define W25N01G_TIMEOUT_RESET_MS            500  // tRSTmax = 500ms

uint8_t AP_Filesystem_FlashMemory_LittleFS::read_status_register()
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[2] = { JEDEC_READ_STATUS, 0 };
    uint8_t status;
    uint8_t length = 1;

    if (jedec_id == JEDEC_ID_WINBOND_W25N01GV) {
        length = 2;
        cmd[1] = W25N01G_STATUS_REG;
    }

    dev->transfer(cmd, length, &status, 1);

    return status;
}

bool AP_Filesystem_FlashMemory_LittleFS::is_busy()
{
    // TODO(ntamas): slightly different for W25N01GV?
    return (read_status_register() & (JEDEC_STATUS_BUSY | JEDEC_STATUS_SRP0)) != 0;
}

void AP_Filesystem_FlashMemory_LittleFS::send_command_addr(uint8_t command, uint32_t addr)
{
    uint8_t cmd[5];
    cmd[0] = command;

    if (use_32bit_address) {
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >>  8) & 0xff;
        cmd[4] = (addr >>  0) & 0xff;
    } else {
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >>  8) & 0xff;
        cmd[3] = (addr >>  0) & 0xff;
        cmd[4] = 0;
    }

    dev->transfer(cmd, use_32bit_address ? 5 : 4, nullptr, 0);
}

void AP_Filesystem_FlashMemory_LittleFS::send_command_page(uint8_t command, uint32_t page)
{
    uint8_t cmd[3];
    cmd[0] = command;
    cmd[1] = (page >> 8) & 0xff;
    cmd[2] = (page >> 0) & 0xff;
    dev->transfer(cmd, 3, nullptr, 0);
}

bool AP_Filesystem_FlashMemory_LittleFS::wait_until_device_is_ready()
{
    if (dead) {
        return false;
    }

    uint32_t t = AP_HAL::millis();
    while (is_busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            mark_dead();
            return false;
        }
    }

    return true;
}

void AP_Filesystem_FlashMemory_LittleFS::write_status_register(uint8_t reg, uint8_t bits)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[3] = { JEDEC_WRITE_STATUS, reg, bits };
    dev->transfer(cmd, 3, nullptr, 0);
}

bool AP_Filesystem_FlashMemory_LittleFS::find_block_size_and_count() {
    if (!wait_until_device_is_ready()) {
        return false;
    }

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4];
    dev->transfer(&cmd, 1, buf, 4);

    jedec_id = buf[0] << 16 | buf[1] << 8 | buf[2];
    
    // For some reason the JEDEC ID is to be found in buf[1], buf[2] and
    // buf[3] for JEDEC_ID_WINBOND_W25N01GV
    if (jedec_id == (JEDEC_ID_WINBOND_W25N01GV >> 8) && buf[3] == (JEDEC_ID_WINBOND_W25N01GV & 0xff)) {
        jedec_id = JEDEC_ID_WINBOND_W25N01GV;
    }

    // Let's specify the terminology here.
    //
    // 1 block = smallest unit that we can _erase_ in a single operation
    // 1 page = smallest unit that we can read or program in a single operation
    //
    // So, for instance, if we have 4K sectors on the flash chip and we can
    // always erase a single 4K sector, the LFS block size will be 4096 bytes,
    // irrespectively of what the flash chip documentation refers to as a "block"

    use_32bit_address = false;
    use_page_data_read_and_write = false;

    /* Most flash chips are programmable in chunks of 256 bytes and erasable in
     * blocks of 4K so we start with these defaults */
    uint32_t block_count = 0;
    uint16_t page_size = 256;
    uint32_t block_size = 4096;

    switch (jedec_id) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        block_count = 32;   /* 128K */
        break;

    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_WINBOND_W25X32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        block_count = 64;   /* 256K */
        break;

    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        block_count = 128;  /* 512K */
        break;

    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_2:
    case JEDEC_ID_CYPRESS_S25FL128L:
        block_count = 256;  /* 1M */
        break;

    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        block_count = 512;  /* 2M */
        use_32bit_address = true;
        break;

    case JEDEC_ID_WINBOND_W25N01GV:
        /* 128M, programmable in chunks of 2048 bytes, erasable in blocks of 128K */
        page_size = 2048;
        block_size = 131072;
        block_count = 1024;
        use_page_data_read_and_write = true;
        break;

    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", jedec_id);
        jedec_id = JEDEC_ID_UNKNOWN;
        return false;
    }

    fs_cfg.read_size = page_size;
    fs_cfg.prog_size = page_size;
    fs_cfg.block_size = block_size;
    fs_cfg.block_count = block_count;

    fs_cfg.block_cycles = 500;
    fs_cfg.lookahead_size = 16;

    // cache_size has to be the same as the page_size, otherwise we would
    // occasionally get read or prog requests in flashmem_read() and
    // flashmem_prog() whose size is equal to the cache size, and we do not
    // handle that right now
    fs_cfg.cache_size = page_size;

    return true;
}

bool AP_Filesystem_FlashMemory_LittleFS::mount_filesystem() {
    if (dead) {
        return false;
    }

    if (mounted) {
        return true;
    }

    fs_cfg.context = this;

    fs_cfg.read = flashmem_read;
    fs_cfg.prog = flashmem_prog;
    fs_cfg.erase = flashmem_erase;
    fs_cfg.sync = flashmem_sync;

    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        mark_dead();
        return false;
    }

    dev_sem = dev->get_semaphore();

    if (!find_block_size_and_count()) {
        mark_dead();
        return false;
    }

    if (!flashmem_init()) {
        mark_dead();
        return false;
    }

    // find_block_size_and_count() filled out the read / prog / block sizes
    // and counts in fs_cfg so it's time to mount

    if (lfs_mount(&fs, &fs_cfg) < 0) {
        /* maybe not formatted? try formatting it */
        printf("FlashMemory_LittleFS: formatting filesystem\n");
        if (lfs_format(&fs, &fs_cfg) < 0) {
            /* cannot format either, give up */
            mark_dead();
            return false;
        }

        /* try mounting again */
        if (lfs_mount(&fs, &fs_cfg) < 0) {
            /* cannot mount after formatting */
            mark_dead();
            return false;
        }
    }

#ifdef HAL_BOARD_STORAGE_DIRECTORY
    // try to create the root storage folder. Ignore the error code in case
    // the filesystem is corrupted or it already exists.
    if (strlen(HAL_BOARD_STORAGE_DIRECTORY) > 0) {
        lfs_mkdir(&fs, HAL_BOARD_STORAGE_DIRECTORY);
    }
#endif

    mounted = true;
    return true;
}

uint32_t AP_Filesystem_FlashMemory_LittleFS::lfs_block_and_offset_to_raw_flash_address(lfs_block_t index, lfs_off_t off)
{
    return index * fs_cfg.block_size + off;
}

uint32_t AP_Filesystem_FlashMemory_LittleFS::lfs_block_to_raw_flash_page_index(lfs_block_t index)
{
    return index * (fs_cfg.block_size / fs_cfg.read_size);
}

bool AP_Filesystem_FlashMemory_LittleFS::flashmem_enable_write()
{
    uint8_t b = JEDEC_WRITE_ENABLE;

    if (!wait_until_device_is_ready()) {
        return false;
    }

    {
        WITH_SEMAPHORE(dev_sem);
        return dev->transfer(&b, 1, nullptr, 0);
    }
}

bool AP_Filesystem_FlashMemory_LittleFS::flashmem_init()
{
    switch (jedec_id) {
        case JEDEC_ID_WINBOND_W25N01GV:
            if (!flashmem_reset()) {
                return false;
            }

            // disable write protection
            write_status_register(W25N01G_PROT_REG, 0);

            // enable ECC and buffer mode
            write_status_register(W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE | W25N01G_CONFIG_BUFFER_READ_MODE);
            break;

        default:
            break;
    }

    return true;
}

bool AP_Filesystem_FlashMemory_LittleFS::flashmem_reset()
{
    uint8_t b = JEDEC_DEVICE_RESET;

    if (!wait_until_device_is_ready()) {
        return false;
    }

    {
        WITH_SEMAPHORE(dev_sem);
        if (!dev->transfer(&b, 1, nullptr, 0)) {
            return false;
        }
    }

    if (jedec_id == JEDEC_ID_WINBOND_W25N01GV) {
        hal.scheduler->delay(W25N01G_TIMEOUT_RESET_MS);
    }

    return true;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_read(
    lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size
) {
    uint32_t address;

    if (dead) {
        return LFS_ERR_IO;
    }

    address = lfs_block_and_offset_to_raw_flash_address(block, off);

    if (use_page_data_read_and_write) {
        /* We need to read an entire page into an internal buffer and then read
         * that buffer with JEDEC_READ_DATA later */
        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }

        {
            WITH_SEMAPHORE(dev_sem);
            send_command_addr(JEDEC_PAGE_DATA_READ, address / fs_cfg.read_size);
            address %= fs_cfg.read_size;
        }
    }

    if (!wait_until_device_is_ready()) {
        return LFS_ERR_IO;
    }

    {
        WITH_SEMAPHORE(dev_sem);

        dev->set_chip_select(true);
        send_command_addr(JEDEC_READ_DATA, address);
        dev->transfer(nullptr, 0, static_cast<uint8_t*>(buffer), size);
        dev->set_chip_select(false);
    }

    return LFS_ERR_OK;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_prog(
    lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size
) {
    uint32_t address;

    if (dead) {
        return LFS_ERR_IO;
    }
    
    if (!wait_until_device_is_ready() || !flashmem_enable_write()) {
        return LFS_ERR_IO;
    }

    address = lfs_block_and_offset_to_raw_flash_address(block, off);
    
    if (use_page_data_read_and_write) {
        /* First we need to write into the data buffer at column address zero,
         * then we need to issue PROGRAM_EXECUTE to commit the internal buffer */
        {
            WITH_SEMAPHORE(dev_sem);

            dev->set_chip_select(true);
            send_command_page(JEDEC_PAGE_WRITE, address % fs_cfg.prog_size);
            dev->transfer(static_cast<const uint8_t*>(buffer), size, nullptr, 0);
            dev->set_chip_select(false);
        }

        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }

        {
            WITH_SEMAPHORE(dev_sem);
            send_command_addr(JEDEC_PROGRAM_EXECUTE, address / fs_cfg.prog_size);
        }

        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }
    } else {
        WITH_SEMAPHORE(dev_sem);

        dev->set_chip_select(true);
        send_command_addr(JEDEC_PAGE_WRITE, address);
        dev->transfer(static_cast<const uint8_t*>(buffer), size, nullptr, 0);
        dev->set_chip_select(false);
    }

    return LFS_ERR_OK;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_erase(lfs_block_t block) {
    if (dead) {
        return LFS_ERR_IO;
    }
    
    if (!wait_until_device_is_ready() || !flashmem_enable_write()) {
        return LFS_ERR_IO;
    }

    {
        WITH_SEMAPHORE(dev_sem);
        if (jedec_id == JEDEC_ID_WINBOND_W25N01GV) {
            send_command_addr(JEDEC_BLOCK_ERASE, lfs_block_to_raw_flash_page_index(block));
        } else {
            send_command_addr(JEDEC_SECTOR4_ERASE, lfs_block_and_offset_to_raw_flash_address(block));
        }
    }

    return LFS_ERR_OK;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_sync() {
    /* Nothing to do, no write caching */
    return LFS_ERR_OK;
}

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_read(block, off, buffer, size);
}

static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_prog(block, off, buffer, size);
}

static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_erase(block);
}

static int flashmem_sync(const struct lfs_config *cfg) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_sync();
}

/* ************************************************************************* */
/* LittleFS to POSIX API conversion functions                                */
/* ************************************************************************* */

static int errno_from_lfs_error(int lfs_error)
{
    switch (lfs_error) {
        case LFS_ERR_OK: return 0;
        case LFS_ERR_IO: return EIO;
        case LFS_ERR_CORRUPT: return EIO;
        case LFS_ERR_NOENT: return ENOENT;
        case LFS_ERR_EXIST: return EEXIST;
        case LFS_ERR_NOTDIR: return ENOTDIR;
        case LFS_ERR_ISDIR: return EISDIR;
        case LFS_ERR_NOTEMPTY: return ENOTEMPTY;
        case LFS_ERR_BADF: return EBADF;
        case LFS_ERR_FBIG: return EFBIG;
        case LFS_ERR_INVAL: return EINVAL;
        case LFS_ERR_NOSPC: return ENOSPC;
        case LFS_ERR_NOMEM: return ENOMEM;
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case LFS_ERR_NOATTR: return ENOATTR;
#endif
        case LFS_ERR_NAMETOOLONG: return ENAMETOOLONG;
        default: return EIO;
    }
}

static int lfs_flags_from_flags(int flags)
{
    int outflags = 0;

    if (flags & O_WRONLY) {
        outflags |= LFS_O_WRONLY;
    } else if (flags & O_RDWR) {
        outflags |= LFS_O_RDWR;
    } else {
        outflags |= LFS_O_RDONLY;
    }

    if (flags & O_CREAT) {
        outflags |= LFS_O_CREAT;
    }

    if (flags & O_EXCL) {
        outflags |= LFS_O_EXCL;
    }
    
    if (flags & O_TRUNC) {
        outflags |= LFS_O_TRUNC;
    }
    
    if (flags & O_APPEND) {
        outflags |= LFS_O_APPEND;
    }
    
    return outflags;
}

#endif  // AP_FILESYSTEM_LITTLEFS_ENABLED
