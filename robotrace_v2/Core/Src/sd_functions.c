/******************************************************************************
 *  File        : sd_functions.c
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


//====================================//
// インクルード
//====================================//
#include "fatfs.h"
#include "sd_diskio_spi.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "ffconf.h"

//====================================//
// グローバル変数の宣言、定義
//====================================//
char sd_path[4];
FATFS fs;
static int sd_linked = 0;

//====================================//
// プロトタイプ宣言
//====================================//

int sd_format(void) {
	// Pre-mount required for legacy FatFS
	f_mount(&fs, sd_path, 0);

	FRESULT res;
	BYTE work[4096]; // working buffer, size must be >= 4 KB

	res = f_mkfs(sd_path, 1, 0, work, sizeof(work));
	if (res != FR_OK) {
		printf("Format failed: f_mkfs returned %d\r\n", res);
	}
		return res;
}

int sd_get_space_kb(void) {
	FATFS *pfs;
	DWORD fre_clust, tot_sect, fre_sect, total_kb, free_kb;
	FRESULT res = f_getfree(sd_path, &fre_clust, &pfs);
	if (res != FR_OK) return res;

	tot_sect = (pfs->n_fatent - 2) * pfs->csize;
	fre_sect = fre_clust * pfs->csize;
	total_kb = tot_sect / 2;
	free_kb = fre_sect / 2;
	printf("💾 Total: %lu KB, Free: %lu KB\r\n", total_kb, free_kb);
	return FR_OK;
}

int sd_mount(void) {
	FRESULT res;
	extern uint8_t sd_is_sdhc(void);

	if (!sd_linked) {
		printf("Linking SD driver...\r\n");
		if (FATFS_LinkDriver(&SD_Driver, sd_path) != 0) {
			printf("FATFS_LinkDriver failed\n");
			return FR_DISK_ERR;
		}
		sd_linked = 1;
	}

	printf("Initializing disk...\r\n");
	DSTATUS stat = disk_initialize(0);
	if (stat != 0) {
		printf("disk_initialize failed: 0x%02X\n", stat);
		printf("FR_NOT_READY\tTry Hard Reset or Check Connection/Power\r\n");
		return FR_NOT_READY;
	}

	printf("Attempting mount at %s...\r\n", sd_path);
	res = f_mount(&fs, sd_path, 1);
	if (res == FR_OK)
	{
		printf("SD card mounted successfully at %s\r\n", sd_path);
		printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

		// Capacity and free space reporting
		sd_get_space_kb();
		return FR_OK;
	}

//	 Handle no filesystem by creating one
	if (res == FR_NO_FILESYSTEM)
	{
		printf("No filesystem found on SD card. Attempting format...\r\nThis will create 32MB Partition (Most Probably)\r\n");
		printf("If you need the full sized SD card, use the computer to format into FAT32\r\n");
		sd_format();

		printf("Retrying mount after format...\r\n");
		res = f_mount(&fs, sd_path, 1);
		if (res == FR_OK) {
			printf("SD card formatted and mounted successfully.\r\n");
			printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

			// Report capacity after format
			sd_get_space_kb();
		}
		else {
			printf("Mount failed even after format: %d\r\n", res);
		}
		return res;
	}

	// Any other mount error
	printf("Mount failed with code: %d\r\n", res);
	return res;
}


int sd_unmount(void) {
	FRESULT res = f_mount(NULL, sd_path, 1);
	printf("SD card unmounted: %s\r\n", (res == FR_OK) ? "OK" : "Failed");
	return res;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 sd_remount
// 処理概要     解析時のI/O再試行用にSDを再初期化する
// 引数         なし
// 戻り値       FatFsの結果コード
/////////////////////////////////////////////////////////////////////
int sd_remount(void) {
	int res = sd_unmount();
	if (res != FR_OK) {
		return res;
	}
	// 読込エラーからの復旧時は空き容量走査と自動フォーマットを行わない。
	if (disk_initialize(0) != 0) {
		return FR_NOT_READY;
	}
	return f_mount(&fs, sd_path, 1);
}
