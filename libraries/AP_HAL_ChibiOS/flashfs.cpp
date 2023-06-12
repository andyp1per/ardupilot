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
 */

#include <hal.h>
#include "lfs.h"
#include "lfs_hal.h"
#include "flashfs.h"
#include "bouncebuffer.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/Semaphores.h>
#include "stm32_util.h"

extern const AP_HAL::HAL& hal;

#ifndef HAL_BOOTLOADER_BUILD
static HAL_Semaphore sem;
#endif
static bool flashfs_running;

static const SNORConfig snorcfg1 = {
  .busp             = &PORTAB_WSPI1,
  .buscfg           = &WSPIcfg1
};

static SNORDriver snor1;
static snor_nocache_buffer_t __nocache_snor1buf;

static uint8_t __lfs_read_buffer[16];
static uint8_t __lfs_prog_buffer[16];
static uint8_t __lfs_lookahead_buffer[16];

static const struct lfs_config lfscfg = {
    /* Link to the flash device driver.*/
    .context            = &snor1,

    /* Block device operations.*/
    .read               = __lfs_read,
    .prog               = __lfs_prog,
    .erase              = __lfs_erase,
    .sync               = __lfs_sync,
    .lock               = __lfs_lock,
    .unlock             = __lfs_unlock,

    /* Block device configuration.*/
    .read_size          = 16,
    .prog_size          = 16,
    .block_size         = 4096,
    .block_count        = 128,
    .block_cycles       = 500,
    .cache_size         = 16,
    .lookahead_size     = 16,
    .read_buffer        = __lfs_read_buffer,
    .prog_buffer        = __lfs_prog_buffer,
    .lookahead_buffer   = __lfs_lookahead_buffer,
    .name_max           = 0,
    .file_max           = 0,
    .attr_max           = 0,
    .metadata_max       = 0
};

static lfs_t lfs;

/*
  initialise microSD card if avaialble. This is called during
  AP_BoardConfig initialisation. The parameter BRD_SD_SLOWDOWN
  controls a scaling factor on the microSD clock
 */
bool flashfs_init()
{
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(sem);
#endif

    /* Initializing and starting snor1 driver.*/
    snorObjectInit(&snor1, &__nocache_snor1buf);
    snorStart(&snor1, &snorcfg1);

    if (sdcd.bouncebuffer == nullptr) {
        // allocate 4k bouncebuffer for microSD to match size in
        // AP_Logger
        bouncebuffer_init(&sdcd.bouncebuffer, 4096, true);
    }

    if (flashfs_running) {
        lfs_unmount(&lfs);
    }

    /* Mounting the file system, if it fails then format.*/
    int err = lfs_mount(&lfs, &lfscfg);
    if (err < 0) {
        err = lfs_format(&lfs, &lfscfg);
        if (err < 0) {
            printf("LFS format failed");
        }
        err = lfs_mount(&lfs, &lfscfg);
        if (err < 0) {
            printf("LFS mount failed")
            flashfs_running = false;
            return false;
        }
    }

    printf("Successfully mounted flashfs\n");

    flashfs_running = true;
    return true;
}

/*
  stop sdcard interface (for reboot)
 */
void flashfs_stop(void)
{
    // unmount
    if (flashfs_running) {
        lfs_unmount(&lfs);
        flashfs_running = false;
    }
}

bool flashfs_retry(void)
{
    if (!flashfs_running) {
        if (flashfs_init()) {
#if HAVE_FILESYSTEM_SUPPORT
            // create APM directory
            AP::FS().mkdir("/APM");
#endif
        }
    }
    return flashfs_running;
}
