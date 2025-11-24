//====================================//
// インクルード
//====================================//
#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // For memcpy
//====================================//
// グローバル変数の宣言
//====================================//
// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

// Screen object
static SSD1306_t SSD1306;

static uint8_t SSD1306_DMA_Page = 0;          // 送信中のページ番号
volatile uint8_t SSD1306_DMA_Completed = 0;   // 全ページ送信完了フラグ
static uint8_t SSD1306_DMA_Running = 0;       // DMA送信継続フラグ
#if defined(SSD1306_USE_I2C)
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Reset
// 処理概要     I2C版リセット処理(実装なし)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Reset(void)
{
	/* for I2C - do nothing */
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteCommand
// 処理概要     コマンドレジスタへの書き込み
// 引数         byte:送信データ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_WriteCommand(uint8_t byte)
{
	HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteData
// 処理概要     データレジスタへの書き込み
// 引数         buffer:送信データポインタ buff_size:送信サイズ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_WriteData(uint8_t *buffer, size_t buff_size)
{
	HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteData_DMA
// 処理概要     DMAを使用したデータ送信
// 引数         buffer:送信データポインタ buff_size:送信サイズ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_WriteData_DMA(uint8_t *buffer, size_t buff_size)
{
	HAL_I2C_Mem_Write_DMA(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size); // DMA送信開始
}
#elif defined(SSD1306_USE_SPI)
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Reset
// 処理概要     SPI版リセット処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Reset(void)
{
	// CS = High (not selected)
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET);

	// Reset the OLED
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteCommand
// 処理概要     コマンドレジスタへの書き込み(SPI)
// 引数         byte:送信データ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_WriteCommand(uint8_t byte)
{
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
	HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_RESET); // command
	HAL_SPI_Transmit(&SSD1306_SPI_PORT, (uint8_t *)&byte, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteData
// 処理概要     データ送信(SPI)
// 引数         buffer:送信データポインタ buff_size:送信サイズ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_WriteData(uint8_t *buffer, size_t buff_size)
{
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
	HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_SET);	// data
	HAL_SPI_Transmit(&SSD1306_SPI_PORT, buffer, buff_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
}
#else
#error "You should define SSD1306_USE_SPI or SSD1306_USE_I2C macro"
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_FillBuffer
// 処理概要     指定バッファ内容を画面バッファへコピー
// 引数         buf:コピー元ポインタ len:コピーサイズ
// 戻り値       SSD1306_OK:成功 SSD1306_ERR:失敗
/////////////////////////////////////////////////////////////////////
SSD1306_Error_t ssd1306_FillBuffer(uint8_t *buf, uint32_t len)
{
	SSD1306_Error_t ret = SSD1306_ERR;
	if (len <= SSD1306_BUFFER_SIZE)
	{
		memcpy(SSD1306_Buffer, buf, len);
		ret = SSD1306_OK;
	}
	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Init
// 処理概要     OLEDの初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Init(void)
{
	// Reset OLED
	ssd1306_Reset();

	// Wait for the screen to boot
	HAL_Delay(100);

	// Init OLED
	ssd1306_SetDisplayOn(0); // display off

	ssd1306_WriteCommand(0x20); // Set Memory Addressing Mode
	ssd1306_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
								// 10b,Page Addressing Mode (RESET); 11b,Invalid

	ssd1306_WriteCommand(0xB0); // Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
	ssd1306_WriteCommand(0xC0); // Mirror vertically
#else
	ssd1306_WriteCommand(0xC8); // Set COM Output Scan Direction
#endif

	ssd1306_WriteCommand(0x00); //---set low column address
	ssd1306_WriteCommand(0x10); //---set high column address

	ssd1306_WriteCommand(0x40); //--set start line address - CHECK

	ssd1306_SetContrast(0xFF);

#ifdef SSD1306_MIRROR_HORIZ
	ssd1306_WriteCommand(0xA0); // Mirror horizontally
#else
	ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
	ssd1306_WriteCommand(0xA7); //--set inverse color
#else
	ssd1306_WriteCommand(0xA6); //--set normal color
#endif

// Set multiplex ratio.
#if (SSD1306_HEIGHT == 128)
	// Found in the Luma Python lib for SH1106.
	ssd1306_WriteCommand(0xFF);
#else
	ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (SSD1306_HEIGHT == 32)
	ssd1306_WriteCommand(0x1F); //
#elif (SSD1306_HEIGHT == 64)
	ssd1306_WriteCommand(0x3F); //
#elif (SSD1306_HEIGHT == 128)
	ssd1306_WriteCommand(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

	ssd1306_WriteCommand(0xA4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content

	ssd1306_WriteCommand(0xD3); //-set display offset - CHECK
	ssd1306_WriteCommand(0x00); //-not offset

	ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0xF0); //--set divide ratio

	ssd1306_WriteCommand(0xD9); //--set pre-charge period
	ssd1306_WriteCommand(0x22); //

	ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (SSD1306_HEIGHT == 32)
	ssd1306_WriteCommand(0x02);
#elif (SSD1306_HEIGHT == 64)
	ssd1306_WriteCommand(0x12);
#elif (SSD1306_HEIGHT == 128)
	ssd1306_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

	ssd1306_WriteCommand(0xDB); //--set vcomh
	ssd1306_WriteCommand(0x20); // 0x20,0.77xVcc

	ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	ssd1306_WriteCommand(0x14); //
	ssd1306_SetDisplayOn(1);	//--turn on SSD1306 panel

	// Clear screen
	ssd1306_Fill(Black);

	// Flush buffer to screen
	ssd1306_UpdateScreen();

	// Set default values for screen object
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	SSD1306.Initialized = 1;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Fill
// 処理概要     画面全体を指定色で塗りつぶす
// 引数         color:塗りつぶし色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Fill(SSD1306_COLOR color)
{
	uint32_t i;

	for (i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_UpdateScreen
// 処理概要     画面バッファをディスプレイへ転送
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_UpdateScreen(void)
{
	// Write data to each page of RAM. Number of pages
	// depends on the screen height:
	//
	//  * 32px   ==  4 pages
	//  * 64px   ==  8 pages
	//  * 128px  ==  16 pages
	for (uint8_t i = 0; i < SSD1306_HEIGHT / 8; i++)
	{
		ssd1306_WriteCommand(0xB0 + i); // Set the current RAM page address.
		ssd1306_WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
		ssd1306_WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
		ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
	}
}
#if defined(SSD1306_USE_I2C)
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_UpdateScreen_DMA
// 処理概要     DMA版画面更新を開始
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_UpdateScreen_DMA(void)
{
	SSD1306_DMA_Page = 0;		// 0ページ目から送信開始
	SSD1306_DMA_Completed = 0;	// 送信開始前に完了フラグをリセット
	SSD1306_DMA_Running = 1;	// DMA送信を有効化

	ssd1306_WriteCommand(0xB0 + SSD1306_DMA_Page); // ページアドレス設定
	ssd1306_WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // 列アドレス下位設定
	ssd1306_WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // 列アドレス上位設定
	ssd1306_WriteData_DMA(&SSD1306_Buffer[SSD1306_WIDTH * SSD1306_DMA_Page], SSD1306_WIDTH); // DMA送信
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_IsTransferCompleted
// 処理概要     全ページ送信完了状態の参照
// 引数         なし
// 戻り値       全ページ送信完了状態
/////////////////////////////////////////////////////////////////////
uint8_t ssd1306_IsTransferCompleted(void)
{
	return SSD1306_DMA_Completed; // 完了フラグを返す
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_StopDMA
// 処理概要     DMA版画面更新を停止
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_StopDMA(void)
{
	SSD1306_DMA_Running = 0;   // DMA送信継続フラグをリセット
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_IsDMARunning
// 処理概要     DMA版画面更新状態の参照
// 引数         なし
// 戻り値       DMA継続状態
/////////////////////////////////////////////////////////////////////
uint8_t ssd1306_IsDMARunning(void)
{
	return SSD1306_DMA_Running;        // DMA継続状態を返す
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_I2C_MemTxCpltCallback
// 処理概要     I2Cメモリ転送完了処理
// 引数         hi2c:I2Cハンドル
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c != &SSD1306_I2C_PORT) // 他デバイスの割り込みは無視
	{
		return;
	}

	if (!SSD1306_DMA_Running) // DMA停止中は再送信しない
	{
		SSD1306_DMA_Completed = 1;           // 完了フラグセット
		ssd1306_TransferCompletedCallback(); // 完了通知
		return;
	}

	SSD1306_DMA_Page++; // 次のページへ

	if (SSD1306_DMA_Page >= SSD1306_HEIGHT / 8)
	{
		SSD1306_DMA_Completed = 1;           // 全ページ送信完了フラグセット
		ssd1306_TransferCompletedCallback(); // 全ページ送信完了通知
		SSD1306_DMA_Page = 0;                // 0ページに戻して継続
	}

	ssd1306_WriteCommand(0xB0 + SSD1306_DMA_Page); // ページアドレス設定
	ssd1306_WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // 列アドレス下位設定
	ssd1306_WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // 列アドレス上位設定
	ssd1306_WriteData_DMA(&SSD1306_Buffer[SSD1306_WIDTH * SSD1306_DMA_Page], SSD1306_WIDTH); // DMA送信
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_TransferCompletedCallback
// 処理概要     転送完了時のコールバック
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
__weak void ssd1306_TransferCompletedCallback(void)
{
	// ユーザー定義の処理をここに追加
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawPixel
// 処理概要     指定座標にピクセルを描画
// 引数         x:X座標 y:Y座標 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
	{
		// Don't write outside the buffer
		return;
	}

	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	}
	else
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteChar
// 処理概要     1文字を画面バッファへ描画
// 引数         ch:文字 Font:フォント情報 color:色
// 戻り値       書き込んだ文字(失敗時0)
/////////////////////////////////////////////////////////////////////
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;

	// Check if character is valid
	if (ch < 32 || ch > 126)
		return 0;

	// Check remaining space on current line
	if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}

	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
			}
			else
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}

	// The current space is now taken
	SSD1306.CurrentX += Font.FontWidth;

	// Return written char for validation
	return ch;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_WriteString
// 処理概要     文字列を画面バッファへ描画
// 引数         str:文字列 Font:フォント情報 color:色
// 戻り値       最後に書き込んだ文字
/////////////////////////////////////////////////////////////////////
char ssd1306_WriteString(char *str, FontDef Font, SSD1306_COLOR color)
{
	while (*str)
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			// Char could not be written
			return *str;
		}
		str++;
	}

	// Everything ok
	return *str;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_SetCursor
// 処理概要     描画位置を設定
// 引数         x:X座標 y:Y座標
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Line
// 処理概要     直線を描画(Bresenham法)
// 引数         x1,y1:始点座標 x2,y2:終点座標 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
	int32_t deltaX = abs(x2 - x1);
	int32_t deltaY = abs(y2 - y1);
	int32_t signX = ((x1 < x2) ? 1 : -1);
	int32_t signY = ((y1 < y2) ? 1 : -1);
	int32_t error = deltaX - deltaY;
	int32_t error2;

	ssd1306_DrawPixel(x2, y2, color);

	while ((x1 != x2) || (y1 != y2))
	{
		ssd1306_DrawPixel(x1, y1, color);
		error2 = error * 2;
		if (error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}

		if (error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_Polyline
// 処理概要     複数頂点から折れ線を描画
// 引数         par_vertex:頂点配列 par_size:頂点数 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
	uint16_t i;
	if (par_vertex == NULL)
	{
		return;
	}

	for (i = 1; i < par_size; i++)
	{
		ssd1306_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
	}

	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DegToRad
// 処理概要     度をラジアンへ変換
// 引数         par_deg:角度(度)
// 戻り値       ラジアン値
/////////////////////////////////////////////////////////////////////
static float ssd1306_DegToRad(float par_deg)
{
	return par_deg * 3.14 / 180.0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_NormalizeTo0_360
// 処理概要     角度を0〜360度に正規化
// 引数         par_deg:角度(度)
// 戻り値       正規化後の角度
/////////////////////////////////////////////////////////////////////
static uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg)
{
	uint16_t loc_angle;
	if (par_deg <= 360)
	{
		loc_angle = par_deg;
	}
	else
	{
		loc_angle = par_deg % 360;
		loc_angle = ((par_deg != 0) ? par_deg : 360);
	}
	return loc_angle;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawArc
// 処理概要     円弧を描画
// 引数         x,y:中心座標 radius:半径 start_angle:開始角度 sweep:角度幅 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
	static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
	float approx_degree;
	uint32_t approx_segments;
	uint8_t xp1, xp2;
	uint8_t yp1, yp2;
	uint32_t count = 0;
	uint32_t loc_sweep = 0;
	float rad;

	loc_sweep = ssd1306_NormalizeTo0_360(sweep);

	count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_degree = loc_sweep / (float)approx_segments;
	while (count < approx_segments)
	{
		rad = ssd1306_DegToRad(count * approx_degree);
		xp1 = x + (int8_t)(sin(rad) * radius);
		yp1 = y + (int8_t)(cos(rad) * radius);
		count++;
		if (count != approx_segments)
		{
			rad = ssd1306_DegToRad(count * approx_degree);
		}
		else
		{
			rad = ssd1306_DegToRad(loc_sweep);
		}
		xp2 = x + (int8_t)(sin(rad) * radius);
		yp2 = y + (int8_t)(cos(rad) * radius);
		ssd1306_Line(xp1, yp1, xp2, yp2, color);
	}

	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawArcWithRadiusLine
// 処理概要     半径線付き円弧を描画
// 引数         x,y:中心座標 radius:半径 start_angle:開始角度 sweep:角度幅 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
	static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
	float approx_degree;
	uint32_t approx_segments;
	uint8_t xp1 = 0;
	uint8_t xp2 = 0;
	uint8_t yp1 = 0;
	uint8_t yp2 = 0;
	uint32_t count = 0;
	uint32_t loc_sweep = 0;
	float rad;

	loc_sweep = ssd1306_NormalizeTo0_360(sweep);

	count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_degree = loc_sweep / (float)approx_segments;

	rad = ssd1306_DegToRad(count * approx_degree);
	uint8_t first_point_x = x + (int8_t)(sin(rad) * radius);
	uint8_t first_point_y = y + (int8_t)(cos(rad) * radius);
	while (count < approx_segments)
	{
		rad = ssd1306_DegToRad(count * approx_degree);
		xp1 = x + (int8_t)(sin(rad) * radius);
		yp1 = y + (int8_t)(cos(rad) * radius);
		count++;
		if (count != approx_segments)
		{
			rad = ssd1306_DegToRad(count * approx_degree);
		}
		else
		{
			rad = ssd1306_DegToRad(loc_sweep);
		}
		xp2 = x + (int8_t)(sin(rad) * radius);
		yp2 = y + (int8_t)(cos(rad) * radius);
		ssd1306_Line(xp1, yp1, xp2, yp2, color);
	}

	// Radius line
	ssd1306_Line(x, y, first_point_x, first_point_y, color);
	ssd1306_Line(x, y, xp2, yp2, color);
	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawCircle
// 処理概要     円を描画(Bresenham法)
// 引数         par_x,par_y:中心座標 par_r:半径 par_color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
	int32_t x = -par_r;
	int32_t y = 0;
	int32_t err = 2 - 2 * par_r;
	int32_t e2;

	if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT)
	{
		return;
	}

	do
	{
		ssd1306_DrawPixel(par_x - x, par_y + y, par_color);
		ssd1306_DrawPixel(par_x + x, par_y + y, par_color);
		ssd1306_DrawPixel(par_x + x, par_y - y, par_color);
		ssd1306_DrawPixel(par_x - x, par_y - y, par_color);
		e2 = err;

		if (e2 <= y)
		{
			y++;
			err = err + (y * 2 + 1);
			if (-x == y && e2 <= x)
			{
				e2 = 0;
			}
		}

		if (e2 > x)
		{
			x++;
			err = err + (x * 2 + 1);
		}
	} while (x <= 0);

	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_FillCircle
// 処理概要     塗りつぶし円を描画(Bresenham法)
// 引数         par_x,par_y:中心座標 par_r:半径 par_color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
	int32_t x = -par_r;
	int32_t y = 0;
	int32_t err = 2 - 2 * par_r;
	int32_t e2;

	if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT)
	{
		return;
	}

	do
	{
		for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--)
		{
			for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--)
			{
				ssd1306_DrawPixel(_x, _y, par_color);
			}
		}

		e2 = err;
		if (e2 <= y)
		{
			y++;
			err = err + (y * 2 + 1);
			if (-x == y && e2 <= x)
			{
				e2 = 0;
			}
		}

		if (e2 > x)
		{
			x++;
			err = err + (x * 2 + 1);
		}
	} while (x <= 0);

	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawRectangle
// 処理概要     矩形を描画
// 引数         x1,y1:左上座標 x2,y2:右下座標 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
	ssd1306_Line(x1, y1, x2, y1, color);
	ssd1306_Line(x2, y1, x2, y2, color);
	ssd1306_Line(x2, y2, x1, y2, color);
	ssd1306_Line(x1, y2, x1, y1, color);

	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_FillRectangle
// 処理概要     矩形を塗りつぶし描画
// 引数         x1,y1:左上座標 x2,y2:右下座標 color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
	uint8_t x_start = ((x1 <= x2) ? x1 : x2);
	uint8_t x_end = ((x1 <= x2) ? x2 : x1);
	uint8_t y_start = ((y1 <= y2) ? y1 : y2);
	uint8_t y_end = ((y1 <= y2) ? y2 : y1);

	for (uint8_t y = y_start; (y <= y_end) && (y < SSD1306_HEIGHT); y++)
	{
		for (uint8_t x = x_start; (x <= x_end) && (x < SSD1306_WIDTH); x++)
		{
			ssd1306_DrawPixel(x, y, color);
		}
	}
	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_DrawBitmap
// 処理概要     ビットマップを描画
// 引数         x,y:描画開始座標 bitmap:ビットマップデータ w:幅 h:高さ color:色
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
	{
		return;
	}

	for (uint8_t j = 0; j < h; j++, y++)
	{
		for (uint8_t i = 0; i < w; i++)
		{
			if (i & 7)
			{
				byte <<= 1;
			}
			else
			{
				byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
			}

			if (byte & 0x80)
			{
				ssd1306_DrawPixel(x + i, y, color);
			}
		}
	}
	return;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_SetContrast
// 処理概要     コントラスト値を設定
// 引数         value:設定値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_SetContrast(const uint8_t value)
{
	const uint8_t kSetContrastControlRegister = 0x81;
	ssd1306_WriteCommand(kSetContrastControlRegister);
	ssd1306_WriteCommand(value);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_SetDisplayOn
// 処理概要     ディスプレイのON/OFFを設定
// 引数         on:1=ON 0=OFF
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_SetDisplayOn(const uint8_t on)
{
	uint8_t value;
	if (on)
	{
		value = 0xAF; // Display on
		SSD1306.DisplayOn = 1;
	}
	else
	{
		value = 0xAE; // Display off
		SSD1306.DisplayOn = 0;
	}
	ssd1306_WriteCommand(value);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_GetDisplayOn
// 処理概要     ディスプレイのON状態取得
// 引数         なし
// 戻り値       1=ON 0=OFF
/////////////////////////////////////////////////////////////////////
uint8_t ssd1306_GetDisplayOn()
{
	return SSD1306.DisplayOn;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_printf
// 処理概要     フォーマット文字列を白色で表示
// 引数         Font:フォント format:書式指定文字列 ...
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_printf(FontDef Font, char *format, ...)
{
	va_list argptr;
	char str[SSD1306_WIDTH / 6 + 10];

	va_start(argptr, format);
	vsprintf(str, format, argptr);
	va_end(argptr);

	ssd1306_WriteString(str, Font, White);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_printfB
// 処理概要     フォーマット文字列を黒色で表示
// 引数         Font:フォント format:書式指定文字列 ...
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ssd1306_printfB(FontDef Font, char *format, ...)
{
	va_list argptr;
	char str[SSD1306_WIDTH / 6 + 10];

	va_start(argptr, format);
	vsprintf(str, format, argptr);
	va_end(argptr);

	ssd1306_WriteString(str, Font, Black);
}