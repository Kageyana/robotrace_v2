/******************************************************************************
 *  File        : sd_diskio_spi.c
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/


#include "diskio.h"
#include "sd_spi.h"
#include "ff_gen_drv.h"

// Debug: last SD read request/status for f_gets error logs
volatile DWORD g_sd_last_read_sector = 0;
volatile UINT g_sd_last_read_count = 0;
volatile int g_sd_last_read_blocks_status = -1;
volatile int g_sd_last_read_multi_status = -1;
static DWORD sd_cached_sector_count = 0;
static uint8_t sd_sector_count_valid = 0;
static DWORD sd_last_fail_sector = 0xFFFFFFFFu;
static uint8_t sd_last_fail_streak = 0;

DSTATUS SD_disk_status(BYTE drv) {
    if (drv != 0)
        return STA_NOINIT;
    return 0;
}

DSTATUS SD_disk_initialize(BYTE drv) {
    if (drv != 0)
        return STA_NOINIT;

    sd_sector_count_valid = 0;
    sd_last_fail_sector = 0xFFFFFFFFu;
    sd_last_fail_streak = 0;
    return (SD_SPI_Init() == SD_OK) ? 0 : STA_NOINIT;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    g_sd_last_read_sector = sector;
    g_sd_last_read_count = count;
    g_sd_last_read_blocks_status = -1;
    g_sd_last_read_multi_status = -1;

    if (pdrv != 0 || count == 0)
        return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;

    SD_Status status = SD_ERROR;
    uint8_t attempt = 0;
    uint8_t max_attempts = 3;

    if (sector == sd_last_fail_sector && sd_last_fail_streak > 0)
    {
        max_attempts = 1;
    }

    while (attempt < max_attempts) {
        status = SD_ReadBlocks(buff, sector, count);
        g_sd_last_read_blocks_status = (int)status;
        if (status == SD_OK) {
            sd_last_fail_sector = 0xFFFFFFFFu;
            sd_last_fail_streak = 0;
            return RES_OK;
        }
        attempt++;
    }

    sd_last_fail_sector = sector;
    if (sd_last_fail_streak < 0xFF)
    {
        sd_last_fail_streak++;
    }
    return RES_ERROR;
}

DRESULT SD_disk_write(BYTE pdrv,  BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    return (SD_WriteBlocks(buff, sector, count) == SD_OK) ? RES_OK : RES_ERROR;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0)
        return RES_PARERR;

    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;
    case GET_SECTOR_COUNT:
        if (!card_initialized) return RES_NOTRDY;
        if (!sd_sector_count_valid) {
            uint32_t sector_count = 0;
            if (SD_GetSectorCount(&sector_count) != SD_OK || sector_count == 0) {
                return RES_ERROR;
            }
            sd_cached_sector_count = (DWORD)sector_count;
            sd_sector_count_valid = 1;
        }
        *(DWORD *)buff = sd_cached_sector_count;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}

const Diskio_drvTypeDef SD_Driver = {
    SD_disk_initialize,
    SD_disk_status,
    SD_disk_read,
#if _USE_WRITE
    SD_disk_write,
#endif
#if _USE_IOCTL
    SD_disk_ioctl,
#endif
};
