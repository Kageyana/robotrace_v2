#include "logBufferWriter.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) \
	do \
	{ \
		if (!(condition)) \
		{ \
			fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
			exit(EXIT_FAILURE); \
		} \
	} while (0)

#define TEST_MAX_BYTES 300000U

typedef struct
{
	uint8_t bytes[TEST_MAX_BYTES];
	uint32_t length;
	uint32_t writeCalls;
} WriteCapture;

static _Alignas(LOG_BUFFER_SECTOR_SIZE_BYTES)
	uint8_t testBuffers[LOG_BUFFER_COUNT][LOG_BUFFER_SIZE_BYTES];

/////////////////////////////////////////////////////////////////////
// モジュール名 drainWrites
// 処理概要     書き込み待ちバッファを回収して出力列を検証する
// 引数         writer: ログバッファ状態 / capture: 書き込み結果
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void drainWrites(LogBufferWriter *writer, WriteCapture *capture)
{
	while (writer->flushPending)
	{
		uint32_t length = writer->flushLength;
		CHECK(length == LOG_BUFFER_SIZE_BYTES);
		CHECK((length % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
		CHECK((capture->length % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
		CHECK(((uintptr_t)writer->flushBuffer % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
		CHECK((capture->length + length) <= TEST_MAX_BYTES);
		memcpy(capture->bytes + capture->length, writer->flushBuffer, length);
		capture->length += length;
		capture->writeCalls++;
		CHECK(logBufferWriterWriteSucceeded(writer, true, length, length));
		logBufferWriterCompleteWrite(writer);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 makeRecord
// 処理概要     レコード番号から検証用の決定的なバイト列を生成する
// 引数         record: 出力先 / recordLength: レコード長[B] / recordNumber: レコード番号
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void makeRecord(uint8_t *record, uint16_t recordLength, uint32_t recordNumber)
{
	for (uint16_t i = 0U; i < recordLength; i++)
	{
		record[i] = (uint8_t)(recordNumber * 31U + (uint32_t)i * 17U);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 prepareTwoPendingAtBoundary
// 処理概要     残り1レコード分のアクティブ領域と2本の書き込み待ちを作る
// 引数         writer: ログバッファ状態 / recordLength: レコード長[B] / expected: 期待バイト列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void prepareTwoPendingAtBoundary(
	LogBufferWriter *writer, uint16_t recordLength, uint8_t *expected)
{
	uint32_t prefixLength = LOG_BUFFER_SIZE_BYTES - recordLength;
	logBufferWriterInit(writer, testBuffers);
	for (uint32_t i = 0U; i < LOG_BUFFER_SIZE_BYTES; i++)
	{
		testBuffers[0][i] = (uint8_t)(0x10U + (i % 101U));
		testBuffers[1][i] = (uint8_t)(0x20U + (i % 97U));
	}
	for (uint32_t i = 0U; i < prefixLength; i++)
	{
		testBuffers[2][i] = (uint8_t)(0x30U + (i % 89U));
	}
	memcpy(expected, testBuffers[0], LOG_BUFFER_SIZE_BYTES);
	memcpy(expected + LOG_BUFFER_SIZE_BYTES, testBuffers[1], LOG_BUFFER_SIZE_BYTES);
	memcpy(expected + (2U * LOG_BUFFER_SIZE_BYTES), testBuffers[2], prefixLength);

	writer->activeBuffer = testBuffers[2];
	writer->activeLength = prefixLength;
	writer->flushBuffer = testBuffers[0];
	writer->flushLength = LOG_BUFFER_SIZE_BYTES;
	writer->flushPending = true;
	writer->pendingBuffer = testBuffers[1];
	writer->pendingLength = LOG_BUFFER_SIZE_BYTES;
	writer->pendingPending = true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 appendExactBoundaryRecord
// 処理概要     残り容量と同じ長さのレコードを満杯バッファへ追加する
// 引数         writer: ログバッファ状態 / recordLength: レコード長[B] / recordNumber: レコード番号 / expected: 期待バイト列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void appendExactBoundaryRecord(
	LogBufferWriter *writer, uint16_t recordLength, uint32_t recordNumber, uint8_t *expected)
{
	uint8_t record[109];
	makeRecord(record, recordLength, recordNumber);
	CHECK(logBufferWriterBeginRecord(writer, recordLength));
	uint16_t typedOffset = (uint16_t)(recordLength - 7U);
	CHECK(logBufferWriterAppendBytes(writer, record, typedOffset));
	CHECK(logBufferWriterAppendU8(writer, record[typedOffset]));
	uint16_t value16 = (uint16_t)(((uint16_t)record[typedOffset + 1U] << 8) |
		record[typedOffset + 2U]);
	CHECK(logBufferWriterAppendU16(writer, value16));
	uint32_t value32 = ((uint32_t)record[typedOffset + 3U] << 24) |
		((uint32_t)record[typedOffset + 4U] << 16) |
		((uint32_t)record[typedOffset + 5U] << 8) | record[typedOffset + 6U];
	CHECK(logBufferWriterAppendU32(writer, value32));
	CHECK(logBufferWriterEndRecord(writer));
	memcpy(expected + (2U * LOG_BUFFER_SIZE_BYTES) + (LOG_BUFFER_SIZE_BYTES - recordLength),
		record, recordLength);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 runProfile
// 処理概要     指定プロファイルで境界またぎと境界一致を検証する
// 引数         recordLength: レコード長[B] / recordCount: レコード数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void runProfile(uint16_t recordLength, uint32_t recordCount)
{
	LogBufferWriter writer;
	WriteCapture capture = {{0U}, 0U, 0U};
	uint32_t expectedLength = (uint32_t)recordLength * recordCount;
	uint8_t *expected = (uint8_t *)malloc(expectedLength);
	CHECK(expected != NULL);
	CHECK(expectedLength <= TEST_MAX_BYTES);

	logBufferWriterInit(&writer, testBuffers);
	CHECK(((uintptr_t)writer.buffers[0] % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK(((uintptr_t)writer.buffers[1] % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK(((uintptr_t)writer.buffers[2] % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);

	uint32_t crossingRecords = 0U;
	uint32_t exactFullRecords = 0U;
	uint8_t record[109];
	for (uint32_t i = 0U; i < recordCount; i++)
	{
		makeRecord(record, recordLength, i);
		uint32_t remaining = LOG_BUFFER_SIZE_BYTES - writer.activeLength;
		if (recordLength > remaining) crossingRecords++;
		if (recordLength == remaining) exactFullRecords++;
		CHECK(logBufferWriterBeginRecord(&writer, recordLength));
		CHECK(logBufferWriterAppendBytes(&writer, record, recordLength));
		CHECK(logBufferWriterEndRecord(&writer));
		memcpy(expected + (i * recordLength), record, recordLength);
		drainWrites(&writer, &capture);
	}

	CHECK(crossingRecords > 0U);
	CHECK(exactFullRecords > 0U);
	CHECK(writer.activeLength == 0U);
	CHECK(capture.length == expectedLength);
	CHECK(memcmp(capture.bytes, expected, expectedLength) == 0);
	CHECK((capture.length % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK(capture.writeCalls == expectedLength / LOG_BUFFER_SIZE_BYTES);

	free(expected);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testFinalPadding
// 処理概要     最終レコード後の512 B境界までのゼロ埋めを検証する
// 引数         recordLength: レコード長[B] / recordCount: レコード数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testFinalPadding(uint16_t recordLength, uint32_t recordCount)
{
	LogBufferWriter writer;
	WriteCapture capture = {{0U}, 0U, 0U};
	uint32_t expectedLength = (uint32_t)recordLength * recordCount;
	uint8_t *expected = (uint8_t *)malloc(expectedLength);
	CHECK(expected != NULL);
	CHECK(expectedLength <= TEST_MAX_BYTES);

	logBufferWriterInit(&writer, testBuffers);
	uint8_t record[109];
	for (uint32_t i = 0U; i < recordCount; i++)
	{
		makeRecord(record, recordLength, i);
		CHECK(logBufferWriterBeginRecord(&writer, recordLength));
		CHECK(logBufferWriterAppendBytes(&writer, record, recordLength));
		CHECK(logBufferWriterEndRecord(&writer));
		memcpy(expected + (i * recordLength), record, recordLength);
		drainWrites(&writer, &capture);
	}

	CHECK(writer.activeLength == recordLength);
	uint32_t finalLength = logBufferWriterPrepareFinalWrite(&writer);
	CHECK(finalLength == LOG_BUFFER_SECTOR_SIZE_BYTES);
	CHECK((capture.length % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK((finalLength % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK(((uintptr_t)writer.activeBuffer % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);
	CHECK((capture.length + finalLength) <= TEST_MAX_BYTES);
	memcpy(capture.bytes + capture.length, writer.activeBuffer, finalLength);
	capture.length += finalLength;
	CHECK(memcmp(capture.bytes, expected, expectedLength) == 0);
	for (uint32_t i = expectedLength; i < capture.length; i++)
	{
		CHECK(capture.bytes[i] == 0U);
	}
	CHECK((capture.length % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U);

	free(expected);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testNoFreeBufferIsAtomic
// 処理概要     空きバッファがないときレコードを追加しないことを検証する
// 引数         recordLength: レコード長[B]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testNoFreeBufferIsAtomic(uint16_t recordLength)
{
	LogBufferWriter writer;
	uint8_t record[109];
	uint32_t acceptedRecords = 0U;
	uint32_t indexBeforeRejectedRecord = 0U;

	logBufferWriterInit(&writer, testBuffers);
	makeRecord(record, sizeof(record), 0U);
	for (uint32_t i = 0U; i < 200U; i++)
	{
		indexBeforeRejectedRecord = writer.activeLength;
		if (!logBufferWriterBeginRecord(&writer, recordLength))
		{
			break;
		}
		CHECK(logBufferWriterAppendBytes(&writer, record, recordLength));
		CHECK(logBufferWriterEndRecord(&writer));
		acceptedRecords++;
	}

	CHECK(writer.overflow);
	CHECK(acceptedRecords > 0U);
	CHECK(writer.flushPending && writer.pendingPending);
	CHECK(writer.activeLength == indexBeforeRejectedRecord);
	CHECK(!logBufferWriterBeginRecord(&writer, recordLength));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testExactFitWithTwoPending
// 処理概要     2本が書き込み待ちでも残り容量に一致するレコードを受け付ける
// 引数         recordLength: レコード長[B]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testExactFitWithTwoPending(uint16_t recordLength)
{
	LogBufferWriter writer;
	uint8_t expected[3U * LOG_BUFFER_SIZE_BYTES + LOG_BUFFER_SECTOR_SIZE_BYTES] = {0U};
	uint8_t record[109];
	prepareTwoPendingAtBoundary(&writer, recordLength, expected);
	appendExactBoundaryRecord(&writer, recordLength, 1U, expected);

	CHECK(writer.activeLength == LOG_BUFFER_SIZE_BYTES);
	CHECK(writer.activeBuffer == testBuffers[2]);
	CHECK(writer.flushPending && writer.pendingPending);
	CHECK(writer.reservedBuffer == NULL);
	CHECK(!writer.overflow);
	makeRecord(record, recordLength, 2U);
	CHECK(!logBufferWriterBeginRecord(&writer, recordLength));
	CHECK(writer.overflow);
	CHECK(writer.activeLength == LOG_BUFFER_SIZE_BYTES);
	CHECK(memcmp(writer.activeBuffer,
		expected + (2U * LOG_BUFFER_SIZE_BYTES), LOG_BUFFER_SIZE_BYTES) == 0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testExactFitResumesInOrder
// 処理概要     書き込み完了後に満杯バッファを順序どおり待ち行列へ移す
// 引数         recordLength: レコード長[B]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testExactFitResumesInOrder(uint16_t recordLength)
{
	LogBufferWriter writer;
	WriteCapture capture = {{0U}, 0U, 0U};
	uint8_t expected[3U * LOG_BUFFER_SIZE_BYTES + LOG_BUFFER_SECTOR_SIZE_BYTES] = {0U};
	uint8_t nextRecord[109];
	prepareTwoPendingAtBoundary(&writer, recordLength, expected);
	appendExactBoundaryRecord(&writer, recordLength, 3U, expected);

	memcpy(capture.bytes, writer.flushBuffer, writer.flushLength);
	capture.length = writer.flushLength;
	CHECK(logBufferWriterWriteSucceeded(&writer, true, LOG_BUFFER_SIZE_BYTES, LOG_BUFFER_SIZE_BYTES));
	logBufferWriterCompleteWrite(&writer);
	CHECK(writer.flushPending && !writer.pendingPending);

	makeRecord(nextRecord, recordLength, 4U);
	CHECK(logBufferWriterBeginRecord(&writer, recordLength));
	CHECK(writer.activeBuffer == testBuffers[0]);
	CHECK(writer.pendingBuffer == testBuffers[2] && writer.pendingPending);
	CHECK(logBufferWriterAppendBytes(&writer, nextRecord, recordLength));
	CHECK(logBufferWriterEndRecord(&writer));
	memcpy(expected + (3U * LOG_BUFFER_SIZE_BYTES), nextRecord, recordLength);
	for (uint32_t i = 0U; i < LOG_BUFFER_SECTOR_SIZE_BYTES; i++)
	{
		if (i >= recordLength)
		{
			expected[(3U * LOG_BUFFER_SIZE_BYTES) + i] = 0U;
		}
	}

	drainWrites(&writer, &capture);
	CHECK(capture.length == 3U * LOG_BUFFER_SIZE_BYTES);
	CHECK(memcmp(capture.bytes, expected, capture.length) == 0);
	uint32_t finalLength = logBufferWriterPrepareFinalWrite(&writer);
	CHECK(finalLength == LOG_BUFFER_SECTOR_SIZE_BYTES);
	memcpy(capture.bytes + capture.length, writer.activeBuffer, finalLength);
	capture.length += finalLength;
	CHECK(capture.length == sizeof(expected));
	CHECK(memcmp(capture.bytes, expected, sizeof(expected)) == 0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testFullActiveFinalWrite
// 処理概要     終了時に保持した満杯バッファ全体を書き込む
// 引数         recordLength: レコード長[B]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testFullActiveFinalWrite(uint16_t recordLength)
{
	LogBufferWriter writer;
	WriteCapture capture = {{0U}, 0U, 0U};
	uint8_t expected[3U * LOG_BUFFER_SIZE_BYTES] = {0U};
	prepareTwoPendingAtBoundary(&writer, recordLength, expected);
	appendExactBoundaryRecord(&writer, recordLength, 5U, expected);
	CHECK(writer.activeLength == LOG_BUFFER_SIZE_BYTES);
	CHECK(!writer.overflow);

	drainWrites(&writer, &capture);
	uint32_t finalLength = logBufferWriterPrepareFinalWrite(&writer);
	CHECK(finalLength == LOG_BUFFER_SIZE_BYTES);
	memcpy(capture.bytes + capture.length, writer.activeBuffer, finalLength);
	capture.length += finalLength;
	CHECK(capture.length == sizeof(expected));
	CHECK(memcmp(capture.bytes, expected, sizeof(expected)) == 0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testShortWriteFails
// 処理概要     短いFatFs書き込みを保存失敗として扱うことを検証する
// 引数         recordLength: レコード長[B]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testShortWriteFails(uint16_t recordLength)
{
	LogBufferWriter writer;
	uint8_t record[109];
	logBufferWriterInit(&writer, testBuffers);
	makeRecord(record, recordLength, 1U);
	for (uint32_t i = 0U; !writer.flushPending && i < 100U; i++)
	{
		CHECK(logBufferWriterBeginRecord(&writer, recordLength));
		CHECK(logBufferWriterAppendBytes(&writer, record, recordLength));
		CHECK(logBufferWriterEndRecord(&writer));
	}
	CHECK(writer.flushPending);
	CHECK(writer.flushLength == LOG_BUFFER_SIZE_BYTES);
	CHECK(!logBufferWriterWriteSucceeded(&writer, true,
		LOG_BUFFER_SIZE_BYTES, LOG_BUFFER_SIZE_BYTES - 1U));
	CHECK(writer.writeFailed);
	CHECK(!writer.flushPending && !writer.pendingPending);
	CHECK(!logBufferWriterBeginRecord(&writer, recordLength));

	logBufferWriterInit(&writer, testBuffers);
	makeRecord(record, recordLength, 2U);
	for (uint32_t i = 0U; !writer.flushPending && i < 100U; i++)
	{
		CHECK(logBufferWriterBeginRecord(&writer, recordLength));
		CHECK(logBufferWriterAppendBytes(&writer, record, recordLength));
		CHECK(logBufferWriterEndRecord(&writer));
	}
	CHECK(writer.flushPending);
	CHECK(!logBufferWriterWriteSucceeded(&writer, false,
		LOG_BUFFER_SIZE_BYTES, LOG_BUFFER_SIZE_BYTES));
	CHECK(writer.writeFailed);
	CHECK(!writer.flushPending && !writer.pendingPending);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testLegacyByteOrder
// 処理概要     数値レコードの従来のビッグエンディアン配置を検証する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testLegacyByteOrder(void)
{
	LogBufferWriter writer;
	const uint8_t expected[7] = {0x12U, 0x34U, 0x56U, 0x78U, 0x9AU, 0xBCU, 0xDEU};
	logBufferWriterInit(&writer, testBuffers);
	CHECK(logBufferWriterBeginRecord(&writer, sizeof(expected)));
	CHECK(logBufferWriterAppendU8(&writer, 0x12U));
	CHECK(logBufferWriterAppendU16(&writer, 0x3456U));
	CHECK(logBufferWriterAppendU32(&writer, 0x789ABCDEU));
	CHECK(logBufferWriterEndRecord(&writer));
	CHECK(memcmp(writer.activeBuffer, expected, sizeof(expected)) == 0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testLegacyBoundarySplit
// 処理概要     16/32bit値がバッファ境界をまたいでも従来順を保つことを検証する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testLegacyBoundarySplit(void)
{
	LogBufferWriter writer;
	uint8_t filler[LOG_BUFFER_SIZE_BYTES - 2U] = {0U};
	const uint8_t expectedLastBytes[2] = {0x12U, 0x34U};
	const uint8_t expectedFirstBytes[4] = {0x56U, 0x78U, 0x9AU, 0xBCU};
	logBufferWriterInit(&writer, testBuffers);
	CHECK(logBufferWriterBeginRecord(&writer, sizeof(filler)));
	CHECK(logBufferWriterAppendBytes(&writer, filler, sizeof(filler)));
	CHECK(logBufferWriterEndRecord(&writer));
	CHECK(logBufferWriterBeginRecord(&writer, 6U));
	CHECK(logBufferWriterAppendU32(&writer, 0x12345678U));
	CHECK(logBufferWriterAppendU16(&writer, 0x9ABCU));
	CHECK(logBufferWriterEndRecord(&writer));
	CHECK(writer.flushPending);
	CHECK(memcmp(writer.flushBuffer + LOG_BUFFER_SIZE_BYTES - 2U,
		expectedLastBytes, sizeof(expectedLastBytes)) == 0);
	CHECK(memcmp(writer.activeBuffer, expectedFirstBytes, sizeof(expectedFirstBytes)) == 0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     通常・詳細プロファイルのログバッファ検証を実行する
// 引数         なし
// 戻り値       成功時はEXIT_SUCCESS
/////////////////////////////////////////////////////////////////////
int main(void)
{
	testLegacyByteOrder();
	testLegacyBoundarySplit();
	testNoFreeBufferIsAtomic(56U);
	testNoFreeBufferIsAtomic(109U);
	testExactFitWithTwoPending(56U);
	testExactFitWithTwoPending(109U);
	testExactFitResumesInOrder(56U);
	testExactFitResumesInOrder(109U);
	testFullActiveFinalWrite(56U);
	testFullActiveFinalWrite(109U);
	testShortWriteFails(56U);
	testShortWriteFails(109U);
	runProfile(56U, 256U);
	runProfile(109U, 2048U);
	testFinalPadding(56U, 257U);
	testFinalPadding(109U, 2049U);
	puts("log sector buffer tests passed");
	return EXIT_SUCCESS;
}
