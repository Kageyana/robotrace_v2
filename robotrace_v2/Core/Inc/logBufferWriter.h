#ifndef LOG_BUFFER_WRITER_H_
#define LOG_BUFFER_WRITER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LOG_BUFFER_SECTOR_SIZE_BYTES 512U
#define LOG_BUFFER_SIZE_BYTES (4U * LOG_BUFFER_SECTOR_SIZE_BYTES)
#define LOG_BUFFER_COUNT 3U

typedef struct
{
	uint8_t (*buffers)[LOG_BUFFER_SIZE_BYTES];
	uint8_t *activeBuffer;
	uint8_t *flushBuffer;
	uint8_t *pendingBuffer;
	uint8_t *reservedBuffer;
	uint32_t activeLength;
	uint32_t flushLength;
	uint32_t pendingLength;
	volatile bool flushPending;
	volatile bool pendingPending;
	volatile bool overflow;
	volatile bool writeFailed;
	bool recordFailed;
} LogBufferWriter;

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterInit
// 処理概要     3本の整列ログバッファとキュー状態を初期化する
// 引数         writer: ログバッファ状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBufferWriterInit(
	LogBufferWriter *writer, uint8_t buffers[][LOG_BUFFER_SIZE_BYTES])
{
	writer->buffers = buffers;
	writer->activeBuffer = writer->buffers[0];
	writer->flushBuffer = writer->buffers[1];
	writer->pendingBuffer = writer->buffers[2];
	writer->reservedBuffer = NULL;
	writer->activeLength = 0U;
	writer->flushLength = 0U;
	writer->pendingLength = 0U;
	writer->flushPending = false;
	writer->pendingPending = false;
	writer->overflow = false;
	writer->writeFailed = false;
	writer->recordFailed = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterFindFreeBuffer
// 処理概要     書き込み中・書き込み待ちではないバッファを探す
// 引数         writer: ログバッファ状態
// 戻り値       空きバッファ。空きがない場合はNULL
/////////////////////////////////////////////////////////////////////
static inline uint8_t *logBufferWriterFindFreeBuffer(LogBufferWriter *writer)
{
	for (uint32_t i = 0U; i < LOG_BUFFER_COUNT; i++)
	{
		uint8_t *buffer = writer->buffers[i];
		if (buffer == writer->activeBuffer || buffer == writer->reservedBuffer)
		{
			continue;
		}
		if (writer->flushPending && buffer == writer->flushBuffer)
		{
			continue;
		}
		if (writer->pendingPending && buffer == writer->pendingBuffer)
		{
			continue;
		}
		return buffer;
	}
	return NULL;
}

static inline bool logBufferWriterRotateFullBuffer(LogBufferWriter *writer);

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterBeginRecord
// 処理概要     満杯バッファを切り替え、境界をまたぐレコードの空きを事前確保する
// 引数         writer: ログバッファ状態 / recordLength: レコード長[B]
// 戻り値       true: 書き込み可能 / false: 書き込み失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterBeginRecord(LogBufferWriter *writer, uint32_t recordLength)
{
	if (writer->writeFailed || writer->overflow || recordLength == 0U ||
		recordLength > LOG_BUFFER_SIZE_BYTES || writer->activeLength > LOG_BUFFER_SIZE_BYTES)
	{
		return false;
	}

	writer->recordFailed = false;
	writer->reservedBuffer = NULL;
	if (writer->activeLength == LOG_BUFFER_SIZE_BYTES)
	{
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
		if (writer->reservedBuffer == NULL || !logBufferWriterRotateFullBuffer(writer))
		{
			writer->overflow = true;
			writer->reservedBuffer = NULL;
			return false;
		}
	}

	uint32_t available = LOG_BUFFER_SIZE_BYTES - writer->activeLength;
	if (recordLength > available)
	{
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
		if (writer->reservedBuffer == NULL)
		{
			writer->overflow = true;
			return false;
		}
	}
	else if (recordLength == available)
	{
		// 満杯時に即時切替できるよう空きがあれば確保する。空きがなくても記録は受け付ける。
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterRotateFullBuffer
// 処理概要     満杯のバッファを待ち行列へ渡して空きバッファへ切り替える
// 引数         writer: ログバッファ状態
// 戻り値       true: 切替成功 / false: 切替失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterRotateFullBuffer(LogBufferWriter *writer)
{
	if (writer->reservedBuffer == NULL || writer->activeLength != LOG_BUFFER_SIZE_BYTES ||
		(writer->flushPending && writer->pendingPending))
	{
		writer->overflow = true;
		writer->recordFailed = true;
		return false;
	}

	if (writer->flushPending)
	{
		writer->pendingBuffer = writer->activeBuffer;
		writer->pendingLength = LOG_BUFFER_SIZE_BYTES;
		writer->pendingPending = true;
	}
	else
	{
		writer->flushBuffer = writer->activeBuffer;
		writer->flushLength = LOG_BUFFER_SIZE_BYTES;
		writer->flushPending = true;
	}

	writer->activeBuffer = writer->reservedBuffer;
	writer->reservedBuffer = NULL;
	writer->activeLength = 0U;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendBytes
// 処理概要     バイト列をバッファ境界で分割して格納する
// 引数         writer: ログバッファ状態 / data: 格納データ / length: データ長[B]
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendBytes(LogBufferWriter *writer, const uint8_t *data, size_t length)
{
	while (length > 0U)
	{
		if (writer->activeLength == LOG_BUFFER_SIZE_BYTES &&
			(writer->reservedBuffer == NULL || !logBufferWriterRotateFullBuffer(writer)))
		{
			writer->recordFailed = true;
			return false;
		}

		uint32_t available = LOG_BUFFER_SIZE_BYTES - writer->activeLength;
		size_t chunkLength = (length < available) ? length : available;
		memcpy(writer->activeBuffer + writer->activeLength, data, chunkLength);
		writer->activeLength += (uint32_t)chunkLength;
		data += chunkLength;
		length -= chunkLength;

		if (writer->activeLength == LOG_BUFFER_SIZE_BYTES && writer->reservedBuffer != NULL &&
			!logBufferWriterRotateFullBuffer(writer))
		{
			writer->recordFailed = true;
			return false;
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterEndRecord
// 処理概要     レコードの格納完了状態を確認する
// 引数         writer: ログバッファ状態
// 戻り値       true: レコード全体を格納 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterEndRecord(LogBufferWriter *writer)
{
	bool complete = writer->reservedBuffer == NULL && !writer->recordFailed;
	if (!complete)
	{
		writer->reservedBuffer = NULL;
		writer->overflow = true;
	}
	writer->recordFailed = false;
	return complete;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU8
// 処理概要     8bit値をログバッファへ格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU8(LogBufferWriter *writer, uint8_t value)
{
	if (writer->activeLength < LOG_BUFFER_SIZE_BYTES)
	{
		writer->activeBuffer[writer->activeLength++] = value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	return logBufferWriterAppendBytes(writer, &value, sizeof(value));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU16
// 処理概要     16bit値を従来のビッグエンディアン形式で格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU16(LogBufferWriter *writer, uint16_t value)
{
	if (writer->activeLength <= LOG_BUFFER_SIZE_BYTES - 2U)
	{
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 8);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	uint8_t bytes[2] = {(uint8_t)(value >> 8), (uint8_t)value};
	return logBufferWriterAppendBytes(writer, bytes, sizeof(bytes));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU32
// 処理概要     32bit値を従来のビッグエンディアン形式で格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU32(LogBufferWriter *writer, uint32_t value)
{
	if (writer->activeLength <= LOG_BUFFER_SIZE_BYTES - 4U)
	{
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 24);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 16);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 8);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	uint8_t bytes[4] = {
		(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value};
	return logBufferWriterAppendBytes(writer, bytes, sizeof(bytes));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterWriteSucceeded
// 処理概要     FatFs書き込み結果を確認し、失敗時は待ち行列を破棄する
// 引数         writer: ログバッファ状態 / ioSucceeded: FatFs結果 / expectedLength: 要求長[B] / writtenLength: 書込長[B]
// 戻り値       true: 全量書き込み成功 / false: 書き込み失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterWriteSucceeded(
	LogBufferWriter *writer, bool ioSucceeded, uint32_t expectedLength, uint32_t writtenLength)
{
	if (ioSucceeded && writtenLength == expectedLength)
	{
		return true;
	}

	writer->writeFailed = true;
	writer->flushPending = false;
	writer->pendingPending = false;
	writer->flushLength = 0U;
	writer->pendingLength = 0U;
	writer->reservedBuffer = NULL;
	return false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterCompleteWrite
// 処理概要     書き込み済みバッファを解放して次の待ちバッファを昇格する
// 引数         writer: ログバッファ状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBufferWriterCompleteWrite(LogBufferWriter *writer)
{
	if (writer->pendingPending)
	{
		writer->flushBuffer = writer->pendingBuffer;
		writer->flushLength = writer->pendingLength;
		writer->pendingPending = false;
		writer->pendingLength = 0U;
		writer->flushPending = true;
	}
	else
	{
		writer->flushPending = false;
		writer->flushLength = 0U;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterPrepareFinalWrite
// 処理概要     残りの有効データを512 B境界までゼロ埋めする
// 引数         writer: ログバッファ状態
// 戻り値       書き込み長[B]。残データなしは0
/////////////////////////////////////////////////////////////////////
static inline uint32_t logBufferWriterPrepareFinalWrite(LogBufferWriter *writer)
{
	if (writer->activeLength == 0U)
	{
		return 0U;
	}

	uint32_t finalLength =
		(writer->activeLength + LOG_BUFFER_SECTOR_SIZE_BYTES - 1U) & ~(LOG_BUFFER_SECTOR_SIZE_BYTES - 1U);
	memset(writer->activeBuffer + writer->activeLength, 0, finalLength - writer->activeLength);
	return finalLength;
}

#endif // LOG_BUFFER_WRITER_H_
