/******************************************************************************
 *  File        : sd_spi.h (SDSC/SDHC support)
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

#ifndef __SD_SPI_H__
#define __SD_SPI_H__

//====================================//
// インクルード
//====================================//
#include "main.h"  // or your specific STM32 family header
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define USE_DMA 1 // SDデータ転送のDMA使用[0:ポーリング、1:DMA]。
#define SD_DMA_WAIT_TIMEOUT_MS 50U // DMA転送完了を待つ上限[ms]。
#define SD_SPI_BYTE_TIMEOUT_MS 20U // SPIの1バイト送受信を待つ上限[ms]。
#ifndef SD_SPI_CRC_CHECK
#define SD_SPI_CRC_CHECK 0 // SDのブロックデータCRC16検査[0:無効、1:有効]。ビルド定義で上書き可能。
#endif
#define SD_SPI_PRESCALER_FAST SPI_BAUDRATEPRESCALER_2 // 通常データ転送時のSPIクロック分周比[2分周]。
#define SD_SPI_PRESCALER_SLOW SPI_BAUDRATEPRESCALER_8 // 低速切替時のSPIクロック分周比[8分周]。

// SDプロトコルのコマンド番号。
#define CMD0  (0)
#define CMD8  (8)
#define CMD9  (9)
#define CMD12 (12)
#define CMD17 (17)
#define CMD18 (18)
#define CMD24 (24)
#define CMD25 (25)
#define CMD55 (55)
#define CMD58 (58)
#define ACMD41 (41)

typedef enum {
    SD_OK = 0,
    SD_ERROR
} SD_Status;

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern uint8_t card_initialized;

//====================================//
// プロトタイプ宣言
//====================================//
SD_Status SD_SPI_Init(void);
SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_GetSectorCount(uint32_t *sector_count);
SD_Status SD_Sync(uint32_t timeout_ms);
uint8_t sd_is_sdhc(void);
bool SD_SetSpiPrescaler(uint32_t prescaler);
void SD_SetSpeedFast(void);
void SD_SetSpeedSlow(void);

#endif // __SD_SPI_H__
