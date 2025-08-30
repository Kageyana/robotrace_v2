//====================================//
// インクルード
//====================================//
#include "fatfs_sd.h"
#include <string.h>
//====================================//
// グローバル変数の宣言
//====================================//
/* manage your SPI handler below */
extern volatile uint8_t Timer1, Timer2; /* 10ms Timer decreasing every time */

static volatile DSTATUS Stat = STA_NOINIT; /* Disc Status Flag*/
static uint8_t CardType;				   /* SD type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0; /* Power condition Flag */
static volatile bool spiTxBusy = false; /* DMA 転送中フラグ */
static volatile bool sdTxSuccess = false; /* 転送完了ステータス */
static uint8_t sdTxBuf[515]; /* トークン+512byte+CRC */

/////////////////////////////////////////////////////////////////////
// モジュール名 SELECT
// 処理概要     SDカードのチップセレクトをアサート
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void SELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DESELECT
// 処理概要     SDカードのチップセレクトを解除
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void DESELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SPI_TxByte
// 処理概要     1バイトをSPIで送信
// 引数         data: 送信するデータ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void SPI_TxByte(BYTE data)
{
	while (HAL_SPI_GetState(&SPI_Handle) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Transmit(&SPI_Handle, &data, 1, SPI_TIMEOUT);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SPI_RxByte
// 処理概要     SPIで1バイト送受信し結果を返す
// 引数         なし
// 戻り値       data: 受信したデータ
/////////////////////////////////////////////////////////////////////
static uint8_t SPI_RxByte(void)
{
	uint8_t dummy, data;
	dummy = 0xFF;
	data = 0;

	while ((HAL_SPI_GetState(&SPI_Handle) != HAL_SPI_STATE_READY))
		;
	HAL_SPI_TransmitReceive(&SPI_Handle, &dummy, &data, 1, SPI_TIMEOUT);

	return data;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SPI_RxBytePtr
// 処理概要     SPIで1バイト受信しバッファへ格納
// 引数         buff: 受信データを書き込むポインタ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void SPI_RxBytePtr(uint8_t *buff)
{
	*buff = SPI_RxByte();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 HAL_SPI_TxCpltCallback
// 処理概要     DMA送信完了割り込みで応答確認とビジー解除を行う
// 引数         hspi: コールバック元ハンドラ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &SPI_Handle)
	{
		uint8_t resp = SPI_RxByte();
		if ((resp & 0x1F) == 0x05)
		{
			while (SPI_RxByte() == 0)
				;
			sdTxSuccess = true;
		}
		else
		{
			sdTxSuccess = false;
		}
		spiTxBusy = false;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_ReadyWait
// 処理概要     SDカードがビジー解除されるまで待機
// 引数         なし
// 戻り値       res: 受信したレスポンス
/////////////////////////////////////////////////////////////////////
static uint8_t SD_ReadyWait(void)
{
	uint8_t res;

	/* 500ms Counter preparation*/
	Timer2 = 50;

	SPI_RxByte();

	do
	{
		/* 0xFF SPI communication until a value is received */
		res = SPI_RxByte();
	} while ((res != 0xFF) && Timer2);

	return res;
}

/*====================================*
 *  DMA を利用した非同期書き込み API
 *====================================*/

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_TxDataBlockAsync
// 処理概要     DMAで512バイトのデータブロックを非同期送信
// 引数         buff: 送信元バッファ token: データトークン
// 戻り値       bool: 送信開始成功=true 送信中/失敗=false
/////////////////////////////////////////////////////////////////////
bool SD_TxDataBlockAsync(const BYTE *buff, BYTE token)
{
	if (spiTxBusy)
		return false;

	sdTxBuf[0] = token;
	memcpy(&sdTxBuf[1], buff, 512);
	sdTxBuf[513] = 0xFF;
	sdTxBuf[514] = 0xFF;

	sdTxSuccess = false;
	spiTxBusy = true;
	HAL_SPI_Transmit_DMA(&SPI_Handle, sdTxBuf, sizeof(sdTxBuf));
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_IsBusy
// 処理概要     DMA送信中かどうかを返す
// 引数         なし
// 戻り値       bool: 送信中=true 非送信=false
/////////////////////////////////////////////////////////////////////
bool SD_IsBusy(void)
{
	return spiTxBusy;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_PowerOn
// 処理概要     SDカードを初期化しSPIモードへ移行
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void SD_PowerOn(void)
{
	uint8_t cmd_arg[6];
	uint32_t Count = 0x1FFF;

	DESELECT();

	for (int i = 0; i < 10; i++)
	{
		SPI_TxByte(0xFF);
	}

	/* SPI Chips Select */
	SELECT();

	/*  GO_IDLE_STATE State transitions*/
	cmd_arg[0] = (CMD0 | 0x40);
	cmd_arg[1] = 0;
	cmd_arg[2] = 0;
	cmd_arg[3] = 0;
	cmd_arg[4] = 0;
	cmd_arg[5] = 0x95;

	/* Command transmission*/
	for (int i = 0; i < 6; i++)
	{
		SPI_TxByte(cmd_arg[i]);
	}

	/* Answer waiting*/
	while ((SPI_RxByte() != 0x01) && Count)
	{
		Count--;
	}

	DESELECT();
	SPI_TxByte(0XFF);

	PowerFlag = 1;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_PowerOff
// 処理概要     SDカードの電源フラグをクリア
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void SD_PowerOff(void)
{
	PowerFlag = 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_CheckPower
// 処理概要     SDカードの電源状態を取得
// 引数         なし
// 戻り値       PowerFlag: 0=off 1=on
/////////////////////////////////////////////////////////////////////
static uint8_t SD_CheckPower(void)
{
	return PowerFlag;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_RxDataBlock
// 処理概要     SDカードからデータパケットを受信
// 引数         buff: 受信バッファ btr: 受信バイト数
// 戻り値       bool: 成功=true 失敗=false
/////////////////////////////////////////////////////////////////////
static bool SD_RxDataBlock(BYTE *buff, UINT btr)
{
	uint8_t token;

	/* 100ms タイマー */
	Timer1 = 10;

	/* 応答待ち */
	do
	{
		token = SPI_RxByte();
	} while ((token == 0xFF) && Timer1);

	/* 0xFE 以外トークン受信時のエラー処理 */
	if (token != 0xFE)
		return FALSE;

	/* バッファへのデータの受信 */
	do
	{
		SPI_RxBytePtr(buff++);
		SPI_RxBytePtr(buff++);
	} while (btr -= 2);

	SPI_RxByte(); /* CRC 無視 */
	SPI_RxByte();

	return TRUE;
}

/* データ転送パケット */
#if _READONLY == 0
/////////////////////////////////////////////////////////////////////
// モジュール名 SD_TxDataBlock
// 処理概要     SDカードへデータパケットを送信
// 引数         buff: 送信バッファ token: データトークン
// 戻り値       bool: 成功=true 失敗=false
/////////////////////////////////////////////////////////////////////
static bool SD_TxDataBlock(const BYTE *buff, BYTE token)
{
	uint8_t resp, wc;
	uint8_t i = 0;

	/* SDカード準備待ち */
	if (SD_ReadyWait() != 0xFF)
		return FALSE;

	/* トークン転送 */
	SPI_TxByte(token);

	/* データトークンの場合 */
	if (token != 0xFD)
	{
		wc = 0;

		/* 512バイトのデータ転送 */
		do
		{
			SPI_TxByte(*buff++);
			SPI_TxByte(*buff++);
		} while (--wc);

		SPI_RxByte(); /* CRC 無視 */
		SPI_RxByte();

		/* データレスポンスの受信 */
		while (i <= 64)
		{
			resp = SPI_RxByte();

			/* エラー応答処理 */
			if ((resp & 0x1F) == 0x05)
				break;

			i++;
		}

		/* SPI 受信バッファ Clear*/
		while (SPI_RxByte() == 0)
			;
	}

	if ((resp & 0x1F) == 0x05)
		return TRUE;
	else
		return FALSE;
}
#endif /* _READONLY */

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_SendCmd
// 処理概要     SDカードへコマンドパケットを送信
// 引数         cmd: コマンド番号 arg: 引数
// 戻り値       res: レスポンスバイト
/////////////////////////////////////////////////////////////////////
static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
	uint8_t crc, res;

	/* SDカード待ち */
	if (SD_ReadyWait() != 0xFF)
		return 0xFF;

	/* コマンドパケット転送 */
	SPI_TxByte(cmd);			   /* Command */
	SPI_TxByte((BYTE)(arg >> 24)); /* Argument[31..24] */
	SPI_TxByte((BYTE)(arg >> 16)); /* Argument[23..16] */
	SPI_TxByte((BYTE)(arg >> 8));  /* Argument[15..8] */
	SPI_TxByte((BYTE)arg);		   /* Argument[7..0] */

	/* コマンドによるCRCの準備 */
	crc = 0;
	if (cmd == CMD0)
		crc = 0x95; /* CRC for CMD0(0) */

	if (cmd == CMD8)
		crc = 0x87; /* CRC for CMD8(0x1AA) */

	/* CRC 転送 */
	SPI_TxByte(crc);

	/* CMD12 Stop Readingコマンドの場合は、応答バイトを1つ捨てる */
	if (cmd == CMD12)
		SPI_RxByte();

	/* 10回以内に正常データを受信する. */
	uint8_t n = 10;
	do
	{
		res = SPI_RxByte();
	} while ((res & 0x80) && --n);

	return res;
}

/*-----------------------------------------------------------------------
  fatfsで使用されるグローバル関数
  user_diskio.cファイルで使用される.
 -----------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_disk_initialize
// 処理概要     SDカードの初期化を行う
// 引数         drv: 物理ドライブ番号
// 戻り値       DSTATUS: ステータスフラグ
/////////////////////////////////////////////////////////////////////
DSTATUS SD_disk_initialize(BYTE drv)
{
	uint8_t n, type, ocr[4];

	/* 一種類のドライブのみサポート */
	if (drv)
		return STA_NOINIT;

	/* SDカード未挿入 */
	if (Stat & STA_NODISK)
		return Stat;

	/* SDカード Power On */
	SD_PowerOn();

	/* SPI 通信のために Chip Select */
	SELECT();

	/* SDカードタイプ変数の初期化 */
	type = 0;

	/* Idle ステータスエントリー */
	if (SD_SendCmd(CMD0, 0) == 1)
	{
		/* タイマー1秒設定 */
		Timer1 = 100;

		/* SD インタフェース動作条件の確認 */
		if (SD_SendCmd(CMD8, 0x1AA) == 1)
		{
			/* SDC Ver2+ */
			for (n = 0; n < 4; n++)
			{
				ocr[n] = SPI_RxByte();
			}

			if (ocr[2] == 0x01 && ocr[3] == 0xAA)
			{
				/* 2.7-3.6V 電圧範囲動作 */
				do
				{
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0)
						break; /* ACMD41 with HCS bit */
				} while (Timer1);

				if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
				{
					/* Check CCS bit */
					for (n = 0; n < 4; n++)
					{
						ocr[n] = SPI_RxByte();
					}

					type = (ocr[0] & 0x40) ? 6 : 2;
				}
			}
		}
		else
		{
			/* SDC Ver1 or MMC */
			type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */

			do
			{
				if (type == 2)
				{
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0)
						break; /* ACMD41 */
				}
				else
				{
					if (SD_SendCmd(CMD1, 0) == 0)
						break; /* CMD1 */
				}
			} while (Timer1);

			if (!Timer1 || SD_SendCmd(CMD16, 512) != 0)
			{
				/* ブロック長の選択 */
				type = 0;
			}
		}
	}

	CardType = type;

	DESELECT();

	SPI_RxByte(); /* Idle 状態遷移 (Release DO) */

	if (type)
	{
		/* Clear STA_NOINIT */
		Stat &= ~STA_NOINIT;
	}
	else
	{
		/* Initialization failed */
		SD_PowerOff();
	}

	return Stat;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_disk_status
// 処理概要     SDカードの状態を取得
// 引数         drv: 物理ドライブ番号
// 戻り値       DSTATUS: ステータスフラグ
/////////////////////////////////////////////////////////////////////
DSTATUS SD_disk_status(BYTE drv)
{
	if (drv)
		return STA_NOINIT;

	return Stat;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_disk_read
// 処理概要     セクタ単位でデータを読み出す
// 引数         pdrv: 物理ドライブ番号 buff: 読み出しバッファ sector: セクタ番号 count: 読み出しセクタ数
// 戻り値       DRESULT: 処理結果
/////////////////////////////////////////////////////////////////////
DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	if (pdrv || !count)
		return RES_PARERR;

	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	if (!(CardType & 4))
		sector *= 512; /* 指定 sector を Byte addressing 単位に変更 */

	SELECT();

	if (count == 1)
	{
		/* シングルブロックを読む */
		if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
			count = 0;
	}
	else
	{
		/* マルチブロック読み取り */
		if (SD_SendCmd(CMD18, sector) == 0)
		{
			do
			{
				if (!SD_RxDataBlock(buff, 512))
					break;

				buff += 512;
			} while (--count);

			/* STOP_TRANSMISSION, すべてのブロックをすべて読んだ後、転送停止要求 */
			SD_SendCmd(CMD12, 0);
		}
	}

	DESELECT();
	SPI_RxByte(); /* Idle 状態(Release DO) */

	return count ? RES_ERROR : RES_OK;
}

/* セクターを書く */
#if _READONLY == 0
/////////////////////////////////////////////////////////////////////
// モジュール名 SD_disk_write
// 処理概要     DMAを利用してセクタ単位でデータを書き込む
// 引数         pdrv: 物理ドライブ番号 buff: 書き込みデータ sector: セクタ番号 count: 書き込みセクタ数
// 戻り値       DRESULT: 処理結果
/////////////////////////////////////////////////////////////////////
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	if (pdrv || !count)
		return RES_PARERR;

	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	if (Stat & STA_PROTECT)
		return RES_WRPRT;

	if (!(CardType & 4))
		sector *= 512; /* 指定 sector を Byte addressing 単位に変更 */

	SELECT();

	if (count == 1)
	{
		/* シングルブロック書き込み */
		if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlockAsync(buff, 0xFE))
		{
			while (SD_IsBusy())
			{
				/* 他処理を実行可能 */
			}

			if (sdTxSuccess)
				count = 0;
		}
	}
	else
	{
		/* マルチブロック書き込み */
		if (CardType & 2)
		{
			SD_SendCmd(CMD55, 0);
			SD_SendCmd(CMD23, count); /* ACMD23 */
		}

		if (SD_SendCmd(CMD25, sector) == 0)
		{
			do
			{
				if (!SD_TxDataBlockAsync(buff, 0xFC))
					break;

				while (SD_IsBusy())
				{
					/* 他処理を実行可能 */
				}

				if (!sdTxSuccess)
					break;

				buff += 512;
			} while (--count);

			if (!SD_TxDataBlock(0, 0xFD))
			{
				count = 1;
			}
		}
	}

	DESELECT();
	SPI_RxByte();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */

/////////////////////////////////////////////////////////////////////
// モジュール名 SD_disk_ioctl
// 処理概要     SDカードに制御コマンドを送る
// 引数         drv: 物理ドライブ番号 ctrl: 制御コード buff: 送受信バッファ
// 戻り値       DRESULT: 処理結果
/////////////////////////////////////////////////////////////////////
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	WORD csize;

	if (drv)
		return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER)
	{
		switch (*ptr)
		{
		case 0:
			if (SD_CheckPower())
				SD_PowerOff(); /* Power Off */
			res = RES_OK;
			break;
		case 1:
			SD_PowerOn(); /* Power On */
			res = RES_OK;
			break;
		case 2:
			*(ptr + 1) = (BYTE)SD_CheckPower();
			res = RES_OK; /* Power Check */
			break;
		default:
			res = RES_PARERR;
		}
	}
	else
	{
		if (Stat & STA_NOINIT)
			return RES_NOTRDY;

		SELECT();

		switch (ctrl)
		{
		case GET_SECTOR_COUNT:
			/* SDカード内のSectorの数 (DWORD) */
			if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
			{
				if ((csd[0] >> 6) == 1)
				{
					/* SDC ver 2.00 */
					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
					*(DWORD *)buff = (DWORD)csize << 10;
				}
				else
				{
					/* MMC or SDC ver 1.XX */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(DWORD *)buff = (DWORD)csize << (n - 9);
				}

				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE:
			/* セクタの単位サイズ (WORD) */
			*(WORD *)buff = 512;
			res = RES_OK;
			break;

		case CTRL_SYNC:
			/* 書き込み同期 */
			if (SD_ReadyWait() == 0xFF)
				res = RES_OK;
			break;

		case MMC_GET_CSD:
			/* CSD情報の受信 (16 bytes) */
			if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID:
			/* CID情報の受信 (16 bytes) */
			if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR:
			/* OCR情報の受信 (4 bytes) */
			if (SD_SendCmd(CMD58, 0) == 0)
			{
				for (n = 0; n < 4; n++)
				{
					*ptr++ = SPI_RxByte();
				}

				res = RES_OK;
			}

		default:
			res = RES_PARERR;
		}

		DESELECT();
		SPI_RxByte();
	}

	return res;
}
