/******************************************************************************
 *  File        : sd_spi.c (SDSC/SDHC support)
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

#include "sd_spi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern volatile int g_sd_last_read_multi_status;

/***************************************************************
 * 🔧 USER-MODIFIABLE SECTION
 * You are free to edit anything below this line
 ***************************************************************/

#define USE_DMA 1

// extern SPI_HandleTypeDef hspi3;
#define SD_SPI_HANDLE hspi3

#define SD_CS_LOW()     HAL_GPIO_WritePin(CS_MSD_GPIO_Port, CS_MSD_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(CS_MSD_GPIO_Port, CS_MSD_Pin, GPIO_PIN_SET)

#ifndef SD_SPI_CRC_CHECK
#define SD_SPI_CRC_CHECK 0
#endif
#define SD_SPI_PRESCALER_FAST SPI_BAUDRATEPRESCALER_2
#define SD_SPI_PRESCALER_SLOW SPI_BAUDRATEPRESCALER_8

/***************************************************************
 * 🚫 DO NOT MODIFY BELOW THIS LINE
 * Auto-generated/system-managed code. Changes may be lost.
 ***************************************************************/

#if USE_DMA
volatile int dma_tx_done = 0;
volatile int dma_rx_done = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &SD_SPI_HANDLE) dma_tx_done = 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &SD_SPI_HANDLE) dma_rx_done = 1;
}
#endif

static void SD_TransmitByte(uint8_t data) {
    HAL_SPI_Transmit(&SD_SPI_HANDLE, &data, 1, HAL_MAX_DELAY);
}

static uint8_t SD_ReceiveByte(void) {
    uint8_t dummy = 0xFF, data = 0;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &dummy, &data, 1, HAL_MAX_DELAY);
    return data;
}

static void SD_TransmitBuffer(const uint8_t *buffer, uint16_t len) {
#if USE_DMA
    dma_tx_done = 0;
    HAL_SPI_Transmit_DMA(&SD_SPI_HANDLE, (uint8_t *)buffer, len);
    while (!dma_tx_done);
#else
    HAL_SPI_Transmit(&SD_SPI_HANDLE, (uint8_t *)buffer, len, HAL_MAX_DELAY);
#endif
}

static void SD_ReceiveBuffer(uint8_t *buffer, uint16_t len) {
#if USE_DMA
	static uint8_t tx_dummy[512];
    for (int i = 0; i < len; i++) tx_dummy[i] = 0xFF;  // Fill with 0xFF
    dma_rx_done = 0;
    HAL_SPI_TransmitReceive_DMA(&SD_SPI_HANDLE, tx_dummy, buffer, len);
    while (!dma_rx_done);
#else
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = SD_ReceiveByte();
    }
#endif
}

static uint16_t SD_CalcCrc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


bool SD_SetSpiPrescaler(uint32_t prescaler) {
    bool ok = false;
    if (HAL_SPI_GetState(&SD_SPI_HANDLE) == HAL_SPI_STATE_BUSY) {
        return false;
    }
    (void)HAL_SPI_DeInit(&SD_SPI_HANDLE);
    SD_SPI_HANDLE.Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(&SD_SPI_HANDLE) == HAL_OK) {
        ok = true;
    }
    return ok;
}

void SD_SetSpeedFast(void) {
    (void)SD_SetSpiPrescaler(SD_SPI_PRESCALER_FAST);
}

void SD_SetSpeedSlow(void) {
    (void)SD_SetSpiPrescaler(SD_SPI_PRESCALER_SLOW);
}

static SD_Status SD_WaitReadyInternal(uint32_t timeout_ms) {
    uint32_t timeout = HAL_GetTick() + timeout_ms;
    uint8_t resp;
    do {
        resp = SD_ReceiveByte();
        if (resp == 0xFF) return SD_OK;
    } while (HAL_GetTick() < timeout);
    return SD_ERROR;
}

static SD_Status SD_WaitReady(void) {
    return SD_WaitReadyInternal(500);
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response, retry = 0xFF;

    if (SD_WaitReady() != SD_OK) {
        return 0xFF;
    }
    SD_TransmitByte(0x40 | cmd);
    SD_TransmitByte(arg >> 24);
    SD_TransmitByte(arg >> 16);
    SD_TransmitByte(arg >> 8);
    SD_TransmitByte(arg);
    SD_TransmitByte(crc);

    do {
        response = SD_ReceiveByte();
    } while ((response & 0x80) && --retry);

    return response;
}

static uint8_t sdhc = 0;
uint8_t sd_is_sdhc(void) {
    return sdhc;
}

SD_Status SD_WaitReadyMs(uint32_t timeout_ms) {
    return SD_WaitReadyInternal(timeout_ms);
}

SD_Status SD_Sync(uint32_t timeout_ms) {
    SD_CS_LOW();
    SD_Status status = SD_WaitReadyInternal(timeout_ms);
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    return status;
}
uint8_t card_initialized = 0;

static uint32_t SD_SectorToAddress(uint32_t sector) {
    return sdhc ? sector : (sector * 512U);
}

static SD_Status SD_WaitDataToken(uint8_t token, uint32_t timeout_ms) {
    uint32_t timeout = HAL_GetTick() + timeout_ms;
    uint8_t resp;
    do {
        resp = SD_ReceiveByte();
        if (resp == token) return SD_OK;
    } while (HAL_GetTick() < timeout);
    return SD_ERROR;
}

static SD_Status SD_StopTransmission(void) {
    uint8_t resp;
    uint8_t retry = 0xFF;

    SD_TransmitByte(0x40 | CMD12);
    SD_TransmitByte(0);
    SD_TransmitByte(0);
    SD_TransmitByte(0);
    SD_TransmitByte(0);
    SD_TransmitByte(0xFF);

    SD_ReceiveByte(); // stuff byte
    do {
        resp = SD_ReceiveByte();
    } while ((resp & 0x80) && --retry);

    if (resp != 0x00) return SD_ERROR;
    return (SD_WaitReady() == SD_OK) ? SD_OK : SD_ERROR;
}

static SD_Status SD_ReadCSD(uint8_t *csd) {
    if (csd == NULL) return SD_ERROR;

    SD_CS_LOW();
    if (SD_SendCommand(CMD9, 0, 0xFF) != 0x00) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_ERROR;
    }

    if (SD_WaitDataToken(0xFE, 200) != SD_OK) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_ERROR;
    }

    SD_ReceiveBuffer(csd, 16);
    SD_ReceiveByte();  // CRC
    SD_ReceiveByte();
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    return SD_OK;
}

SD_Status SD_SPI_Init(void) {
    uint8_t i, response;
    uint8_t r7[4];
    uint32_t retry;

    SD_SetSpeedSlow();
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    for (i = 0; i < 10; i++) SD_TransmitByte(0xFF);

    SD_CS_LOW();
    response = SD_SendCommand(CMD0, 0, 0x95);
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    if (response != 0x01) return SD_ERROR;

    SD_CS_LOW();
    response = SD_SendCommand(CMD8, 0x000001AA, 0x87);
    for (i = 0; i < 4; i++) r7[i] = SD_ReceiveByte();
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);

    sdhc = 0;
    retry = HAL_GetTick() + 1000;
    if (response == 0x01 && r7[2] == 0x01 && r7[3] == 0xAA) {
        do {
            SD_CS_LOW();
            SD_SendCommand(CMD55, 0, 0xFF);
            response = SD_SendCommand(ACMD41, 0x40000000, 0xFF);
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
        } while (response != 0x00 && HAL_GetTick() < retry);

        if (response != 0x00) return SD_ERROR;

        SD_CS_LOW();
        response = SD_SendCommand(CMD58, 0, 0xFF);
        uint8_t ocr[4];
        for (i = 0; i < 4; i++) ocr[i] = SD_ReceiveByte();
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        if (ocr[0] & 0x40) sdhc = 1;
    } else {
        do {
            SD_CS_LOW();
            SD_SendCommand(CMD55, 0, 0xFF);
            response = SD_SendCommand(ACMD41, 0, 0xFF);
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
        } while (response != 0x00 && HAL_GetTick() < retry);
        if (response != 0x00) return SD_ERROR;
    }

    card_initialized = 1;
    SD_SetSpeedFast();
    return SD_OK;
}

SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;

    if (count == 1) {
        uint32_t addr = SD_SectorToAddress(sector);
        SD_CS_LOW();
        if (SD_SendCommand(CMD17, addr, 0xFF) != 0x00) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }

        if (SD_WaitDataToken(0xFE, 200) != SD_OK) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }

        SD_ReceiveBuffer(buff, 512);
        uint8_t crc_hi = SD_ReceiveByte();
        uint8_t crc_lo = SD_ReceiveByte();
#if SD_SPI_CRC_CHECK
        uint16_t crc = SD_CalcCrc16(buff, 512);
        if ((((uint16_t)crc_hi << 8) | crc_lo) != crc) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }
#endif
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_OK;
    } else {
        return SD_ReadMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) {
        g_sd_last_read_multi_status = (int)SD_ERROR;
        return SD_ERROR;
    }
    uint32_t addr = SD_SectorToAddress(sector);

    SD_CS_LOW();
    if (SD_SendCommand(CMD18, addr, 0xFF) != 0x00) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        g_sd_last_read_multi_status = (int)SD_ERROR;
        return SD_ERROR;
    }

    while (count--) {
        if (SD_WaitDataToken(0xFE, 200) != SD_OK) {
            (void)SD_StopTransmission();
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            g_sd_last_read_multi_status = (int)SD_ERROR;
            return SD_ERROR;
        }

        SD_ReceiveBuffer(buff, 512);
        uint8_t crc_hi = SD_ReceiveByte();
        uint8_t crc_lo = SD_ReceiveByte();
#if SD_SPI_CRC_CHECK
        uint16_t crc = SD_CalcCrc16(buff, 512);
        if ((((uint16_t)crc_hi << 8) | crc_lo) != crc) {
            (void)SD_StopTransmission();
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            g_sd_last_read_multi_status = (int)SD_ERROR;
            return SD_ERROR;
        }
#endif

        buff += 512;
    }

    if (SD_StopTransmission() != SD_OK) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        g_sd_last_read_multi_status = (int)SD_ERROR;
        return SD_ERROR;
    }
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);

    g_sd_last_read_multi_status = (int)SD_OK;
    return SD_OK;
}

SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;

    if (count == 1) {
        uint32_t addr = SD_SectorToAddress(sector);
        SD_CS_LOW();
        if (SD_SendCommand(CMD24, addr, 0xFF) != 0x00) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }

        SD_TransmitByte(0xFE);
        SD_TransmitBuffer(buff, 512);
        SD_TransmitByte(0xFF);
        SD_TransmitByte(0xFF);

        uint8_t resp = SD_ReceiveByte();
        if ((resp & 0x1F) != 0x05) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }

        if (SD_WaitReady() != SD_OK) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);

        return SD_OK;
    } else {
        return SD_WriteMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;
    uint32_t addr = SD_SectorToAddress(sector);

    SD_CS_LOW();
    if (SD_SendCommand(CMD25, addr, 0xFF) != 0x00) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_ERROR;
    }

    while (count--) {
        SD_TransmitByte(0xFC);  // Start multi-block write token

        SD_TransmitBuffer((uint8_t *)buff, 512);
        SD_TransmitByte(0xFF);  // dummy CRC
        SD_TransmitByte(0xFF);

        uint8_t resp = SD_ReceiveByte();
        if ((resp & 0x1F) != 0x05) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }

        if (SD_WaitReady() != SD_OK) {
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
            return SD_ERROR;
        }
        buff += 512;
    }

    SD_TransmitByte(0xFD);  // STOP_TRAN token
    if (SD_WaitReady() != SD_OK) {
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_ERROR;
    }

    SD_CS_HIGH();

    SD_TransmitByte(0xFF);

    return SD_OK;
}

SD_Status SD_GetSectorCount(uint32_t *sector_count) {
    uint8_t csd[16];
    uint8_t csd_structure;
    uint32_t c_size;

    if (sector_count == NULL) return SD_ERROR;
    if (SD_ReadCSD(csd) != SD_OK) return SD_ERROR;

    csd_structure = (csd[0] >> 6) & 0x03;
    if (csd_structure == 1) {
        c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
               | ((uint32_t)csd[8] << 8)
               | (uint32_t)csd[9];
        *sector_count = (c_size + 1U) * 1024U;
        return SD_OK;
    } else if (csd_structure == 0) {
        uint32_t c_size_mult;
        uint32_t read_bl_len;
        uint32_t block_len;
        uint32_t mult;
        uint32_t block_count;

        c_size = ((uint32_t)(csd[6] & 0x03) << 10)
               | ((uint32_t)csd[7] << 2)
               | ((uint32_t)(csd[8] & 0xC0) >> 6);
        c_size_mult = ((uint32_t)(csd[9] & 0x03) << 1)
                    | ((uint32_t)(csd[10] & 0x80) >> 7);
        read_bl_len = (uint32_t)(csd[5] & 0x0F);
        if (read_bl_len >= 31U) return SD_ERROR;

        block_len = 1U << read_bl_len;
        if (block_len < 512U) return SD_ERROR;

        mult = 1U << (c_size_mult + 2U);
        block_count = (c_size + 1U) * mult;
        *sector_count = block_count * (block_len / 512U);
        return SD_OK;
    }

    return SD_ERROR;
}
