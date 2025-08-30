#ifndef __FATFS_SD_H
#define __FATFS_SD_H
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "stm32f4xx_hal.h"
#include "diskio.h"
#include <stdbool.h>
//======================================//
// マクロ定義
//======================================//
/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x40+8)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */

DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_TxDataBlockAsync
// 処理概要     DMAで512バイトのデータブロックを非同期送信
// 引数         buff: 送信元バッファ token: データトークン
// 戻り値       bool: 送信開始成功=true 送信中/失敗=false
/////////////////////////////////////////////////////////////////////
bool SD_TxDataBlockAsync(const BYTE *buff, BYTE token);

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_IsBusy
// 処理概要     DMA送信中かどうかを返す
// 引数         なし
// 戻り値       bool: 送信中=true 非送信=false
/////////////////////////////////////////////////////////////////////
bool SD_IsBusy(void);

#define SPI_TIMEOUT 1000

#define TRUE  1
#define FALSE 0
//#define bool BYTE

/* defines for the CS PIN */
#define SD_CS_GPIO_Port CS_MSD_GPIO_Port
#define SD_CS_Pin CS_MSD_Pin
/* manage your SPI handler below */
#define SPI_Handle hspi3
//======================================//
// プロトタイプ宣言
//======================================//

#endif
