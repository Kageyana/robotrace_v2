// 現行binの検証・型付き読込・メタデータ解析を担当する。
// 保存時の全件検証も同じ読込処理を使用し、I/O領域はSDcard.cと共有する。
//====================================//
// インクルード
//====================================//
#include "logSource.h"
#include "logStorageInternal.h"
#include "SDcard.h"
#include "courseAnalysis.h"
#include "calibrationConstants.h"
#include "markerSensor.h"
#include "logPerformance.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CROSSSEG_MAX 128
#define logBuffers (logIoMemory.logging)
enum { LOG_READ_RECORD_BYTES = LOG_RECORD_SIZE_BYTES };
//====================================//
// グローバル変数の宣言、定義
//====================================//
static const uint8_t *logaddress;
static float cross_start_mm[CROSSSEG_MAX];
static float cross_end_mm[CROSSSEG_MAX];
static uint16_t sourceCrossCount;
static struct { FIL *file; FSIZE_t end, physical; UINT position, length; } logReadAhead;

// スキーマ順で生成するレコード配置。
typedef struct
{
#define LOG_STRUCT_FIELD(type, name, fmt, expr) LOG_CTYPE_##type name;
#define LOG_STRUCT_SKIP(type, name, fmt, expr)
	LOG_FIELD_LIST(LOG_STRUCT_FIELD, LOG_STRUCT_SKIP)
#undef LOG_STRUCT_FIELD
#undef LOG_STRUCT_SKIP
} LogRecord;

//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadU8
// 処理概要     logaddressから1バイト読み出してポインタを進める
// 引数         なし
// 戻り値       読み出した8bit値
/////////////////////////////////////////////////////////////////////
static uint8_t logReadU8(void)
{
	return *logaddress++;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadU16
// 処理概要     logaddressから16bitをビッグエンディアンで読み出す
// 引数         なし
// 戻り値       読み出した16bit値
/////////////////////////////////////////////////////////////////////
static uint16_t logReadU16(void)
{
	uint16_t hi = (uint16_t)*logaddress++;
	uint16_t lo = (uint16_t)*logaddress++;
	return (uint16_t)((hi << 8) | lo);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadS16
// 処理概要     16bitを読み出して符号付きへ変換する
// 引数         なし
// 戻り値       読み出した16bitの符号付き値
/////////////////////////////////////////////////////////////////////
static int16_t logReadS16(void)
{
	return (int16_t)logReadU16();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadU32
// 処理概要     logaddressから32bitをビッグエンディアンで読み出す
// 引数         なし
// 戻り値       読み出した32bit値
/////////////////////////////////////////////////////////////////////
static uint32_t logReadU32(void)
{
	uint32_t value = 0;

	value = (uint32_t)*logaddress++ << 24;
	value |= (uint32_t)*logaddress++ << 16;
	value |= (uint32_t)*logaddress++ << 8;
	value |= (uint32_t)*logaddress++;
	return value;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadS32
// 処理概要     32bitを読み出して符号付きへ変換する
// 引数         なし
// 戻り値       読み出した32bitの符号付き値
/////////////////////////////////////////////////////////////////////
static int32_t logReadS32(void)
{
	return (int32_t)logReadU32();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadF32
// 処理概要     32bitの生ビットをfloatへ変換する
// 引数         なし
// 戻り値       読み出したfloat値
/////////////////////////////////////////////////////////////////////
static float logReadF32(void)
{
	union
	{
		float f;
		uint32_t i;
	} ftoi;

	ftoi.i = logReadU32();
	return ftoi.f;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadRecord
// 処理概要     LOG_FIELD_LISTの順で1レコードを復元する
// 引数         rec: 現行ビルドのレコード復元先
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logReadRecord(LogRecord *rec)
{
    memset(rec, 0, sizeof(*rec));
#define LOG_READ_FIELD(type, name, fmt, expr) rec->name = logRead##type();
#define LOG_READ_SKIP(type, name, fmt, expr)
    LOG_FIELD_LIST(LOG_READ_FIELD, LOG_READ_SKIP)
#undef LOG_READ_FIELD
#undef LOG_READ_SKIP
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadAheadReset
// 処理概要     seek・再open・closeで先読み状態を破棄する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logReadAheadReset(void)
{
    memset(&logReadAhead, 0, sizeof(logReadAhead));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadBuffered
// 処理概要     記録部終端を超えず4KiB先読みから指定長を復元する
// 引数         file: 入力, bytes: 出力, length: 復元長[B], end: 記録部終端[B]
// 戻り値       FatFs結果（短い読込もエラー）
/////////////////////////////////////////////////////////////////////
static FRESULT logReadBuffered(FIL *file, uint8_t *bytes, UINT length, FSIZE_t end)
{
    if (logReadAhead.file != file || logReadAhead.end != end || logReadAhead.physical != f_tell(file)) {
        logReadAheadReset();
        logReadAhead.file = file; logReadAhead.end = end; logReadAhead.physical = f_tell(file);
    }
    while (length != 0U) {
        if (logReadAhead.position == logReadAhead.length) {
            FSIZE_t offset = f_tell(file);
            if (offset >= end) { logReadAheadReset(); return FR_DISK_ERR; }
            UINT request = end - offset > sizeof(logIoMemory.stopped.input) ?
                (UINT)sizeof(logIoMemory.stopped.input) : (UINT)(end - offset);
            UINT received = 0U;
            LOG_PERF_BEGIN(tick);
            FRESULT result = f_read(file, logIoMemory.stopped.input, request, &received);
            LOG_PERF_END(readMs, tick); LOG_PERF_ADD(readCalls, 1U); LOG_PERF_ADD(readBytes, received);
            if (result != FR_OK || received != request) {
                logReadAheadReset(); return result == FR_OK ? FR_DISK_ERR : result;
            }
            logReadAhead.position = 0U; logReadAhead.length = received; logReadAhead.physical = f_tell(file);
        }
        UINT count = logReadAhead.length - logReadAhead.position;
        if (count > length) count = length;
        memcpy(bytes, logIoMemory.stopped.input + logReadAhead.position, count);
        logReadAhead.position += count; bytes += count; length -= count;
    }
    return FR_OK;
}

/////////////////////////////////////////////////////////////////////
// バイナリの保存・検証
/////////////////////////////////////////////////////////////////////
// モジュール名 logStorageScanRecords
// 処理概要     記録のCRCとクロス区間を取得し、保存時は一次経路も検証する
// 引数         file: 入力, rows: 行数, saved: 保存時状態(NULLは読込), validation: 保存検証結果, dataCrc: CRC出力
// 戻り値       true: 読込成功 false: 読込失敗
/////////////////////////////////////////////////////////////////////
bool logStorageScanRecords(FIL *file, uint16_t rows, const LogSaveCheck *saved,
    PrimaryRouteValidation *validation, uint32_t *dataCrc)
{
	const bool live = saved != NULL;
	uint8_t log[LOG_READ_RECORD_BYTES];
	LogRecord rec;
	FRESULT fresult;
	uint16_t j, time, beforeTime = 0, cross_count = 0;
	int16_t speed, beforeSpeed = 0;
	float dt, dist_mm = 0.0F, goalMarkerX = 0.0F;
	bool in_cross = false, goalMarkerBracketFound = false, totalPulseMonotonic = true;
	int32_t goalMarkerPulse = 0, previousTotalPulse = 0;
	int64_t correctedPulseTotal = 0;
	uint32_t crc = UINT32_MAX;
	logReadAheadReset();
	if (f_lseek(file, LOG_BINARY_HEADER_BYTES) != FR_OK) return false;
	LOG_PERF_BEGIN(scanTick); LOG_PERF_ADD(fullScans, 1U);
	// クロスライン前後100mm直線化のため、2パスで補正する
	// pass1: 距離基準でクロスライン区間を抽出
	beforeTime = 0;
	beforeSpeed = 0;
	dist_mm = 0.0f;
	in_cross = false;
	clearXYcie();
	if (live) *validation = (PrimaryRouteValidation){false, false, false, 0, 0, 0.0F, 1U};
	if (live) validation->goalMarkerOnsetValid =
		markerGetGoalOnsetPulse(&goalMarkerPulse) && goalMarkerPulse > 0;
	if (live) validation->goalMarkerOnset_p = goalMarkerPulse;
	for (j = 0; j < rows; j++)
	{
		fresult = logReadBuffered(file, log, LOG_SIZE, LOG_BINARY_HEADER_BYTES + (FSIZE_t)rows * LOG_SIZE);
		if (fresult != FR_OK)
		{
			printf("f_read error in endLog\r\n");
			return false;
		}
		crc = logBinaryCrcUpdate(crc, log, LOG_SIZE);

		logaddress = log;
		logReadRecord(&rec);

		time = rec.cntlog;
		speed = (int16_t)rec.encCurrentN;
		int32_t totalPulse = (int32_t)rec.encTotalOptimal;
		int32_t correctedPulse = (int32_t)rec.encCurrentCorr_p;
		int32_t pulseDelta = totalPulse - previousTotalPulse;
		if (pulseDelta < 0 || correctedPulse < 0 || correctedPulse > INT16_MAX)
		{
			totalPulseMonotonic = false;
		}
		// encCurrentCorr_p は1ms当たりのパルス数。ログ間隔[ms]を掛けて累積距離と比較する。
		if (correctedPulse >= 0) correctedPulseTotal += (int64_t)correctedPulse * (time - beforeTime);
		if (abs((int32_t)speed - (int32_t)beforeSpeed) > 500)
		{
			speed = beforeSpeed;
		}
		beforeSpeed = speed;
		dt = (float)(time - beforeTime) / 1000.0f;
		float previousX = xycie.x;
		if (live && correctedPulse >= 0 && correctedPulse <= INT16_MAX)
		{
			calcXYcie((int64_t)rec.encTotalL + rec.encTotalR, rec.imuAngle_Z);
		}
		if (live && !goalMarkerBracketFound && validation->goalMarkerOnsetValid &&
			pulseDelta >= 0 && goalMarkerPulse >= previousTotalPulse && goalMarkerPulse <= totalPulse)
		{
			float ratio = (pulseDelta > 0) ?
				(float)(goalMarkerPulse - previousTotalPulse) / (float)pulseDelta : 1.0F;
			if (ratio < 0.0F) ratio = 0.0F;
			if (ratio > 1.0F) ratio = 1.0F;
			goalMarkerX = previousX + ((xycie.x - previousX) * ratio);
			goalMarkerBracketFound = true;
		}
		previousTotalPulse = totalPulse;
		dist_mm += calcDlMm(speed, dt);
		beforeTime = time;

		bool is_cross = (rec.courseMarker == 3);
		if (!in_cross && is_cross)
		{
			if (cross_count < CROSSSEG_MAX)
			{
				cross_start_mm[cross_count] = dist_mm;
			}
			in_cross = true;
		}
		else if (in_cross && !is_cross)
		{
			if (cross_count < CROSSSEG_MAX)
			{
				cross_end_mm[cross_count] = dist_mm;
				cross_count++;
			}
			in_cross = false;
		}
	}
	if (in_cross && cross_count < CROSSSEG_MAX)
	{
		cross_end_mm[cross_count] = dist_mm;
		cross_count++;
	}

	sourceCrossCount = cross_count;
	*dataCrc = ~crc;
	LOG_PERF_END(scanMs, scanTick);
	if (!live) return true;
	validation->goalMarkerX_mm = goalMarkerX;
	if (optimalTrace == BOOST_NONE)
	{
		if (!validation->goalMarkerOnsetValid || SGmarker < COUNT_GOAL)
		{
			validation->reason = 2U;
		}
		else if (logOverflow || markerOverflow || saved->writeFailed || dbg_overflow != 0U ||
			emcStop != 0U || j != rows)
		{
			validation->reason = 3U;
		}
		else if (!saved->imuCalibrationValid ||
			saved->imuCalibrationSamples != CALIBRATED_IMU_SAMPLE_COUNT ||
			saved->imuCalibrationReadErrors != 0U)
		{
			validation->reason = 9U;
		}
		else if (!goalMarkerBracketFound || goalMarkerPulse <= 0)
		{
			validation->reason = 5U;
		}
		else
		{
			int64_t scaleError = (int64_t)previousTotalPulse - correctedPulseTotal;
			int64_t scaleErrorAbs = (scaleError < 0) ? -scaleError : scaleError;
			int64_t scaleTolerance = (int64_t)lroundf(fmaxf(
				50.0F * PULSE_MILLIMETER, (float)previousTotalPulse * 0.05F));
			if (scaleError > INT32_MAX) scaleError = INT32_MAX;
			if (scaleError < INT32_MIN) scaleError = INT32_MIN;
			validation->distanceScaleError_p = (int32_t)scaleError;
			validation->distanceScaleVerified =
				totalPulseMonotonic && previousTotalPulse > 0 && scaleErrorAbs <= scaleTolerance &&
				PULSE_METER == CALIBRATED_ENCODER_PULSE_PER_METER;
			if (!validation->distanceScaleVerified)
			{
				validation->reason = 10U;
			}
			else if (!isfinite(goalMarkerX) || fabsf(goalMarkerX) > PRIMARY_ROUTE_GOAL_X_LIMIT_MM)
			{
				validation->reason = 8U;
			}
			else
			{
				validation->reason = 0U;
				validation->closureValid = true;
			}
		}
	}
	else
	{
		validation->reason = 1U;
	}
	return true;
}

static struct {
    FIL *file;
    LogBinaryHeader header;
    LogSourcePurpose purpose;
    LogRecord record;
    uint32_t row, crc;
    uint16_t previousTime;
    int16_t previousSpeed;
    float distanceMm, roc, x, y;
    FRESULT error;
} binarySource;

/////////////////////////////////////////////////////////////////////
// モジュール名 logStorageSchemaCrc
// 処理概要     列名・型・順序・派生計算版からスキーマ識別CRCを取得する
// 引数         なし
// 戻り値       スキーマ識別CRC
/////////////////////////////////////////////////////////////////////
uint32_t logStorageSchemaCrc(void)
{
    return ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)logSchemaText, sizeof(logSchemaText) - 1U);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logStorageReadContainerHeader
// 処理概要     保存済みコンテナのヘッダ・形式・ファイル長を検証する
// 引数         file: ファイル, number: 期待番号, header: 出力先
// 戻り値       true: 有効 false: 読込失敗または未対応形式
/////////////////////////////////////////////////////////////////////
bool logStorageReadContainerHeader(FIL *file, uint16_t number, LogBinaryHeader *header)
{
    UINT read = 0U;
    if (f_read(file, logBuffers[0], LOG_BINARY_HEADER_BYTES, &read) != FR_OK ||
        read != LOG_BINARY_HEADER_BYTES || !logBinaryDecodeHeader(logBuffers[0], f_size(file), header) ||
        header->number != number || header->profile != LOG_SCHEMA_PROFILE_LIGHT ||
        header->recordBytes != LOG_SIZE || header->schema != logStorageSchemaCrc() ||
        header->schemaBytes != sizeof(logSchemaText) - 1U || header->columns != LOG_BINARY_COLUMNS) return false;
    // 機体の計画解析は現在のビルドと同じ列定義を要求する。PCは列定義から汎用復元する。
    logReadAheadReset();
    if (f_lseek(file, LOG_BINARY_HEADER_BYTES + (FSIZE_t)header->rows * header->recordBytes) != FR_OK ||
        f_read(file, logIoMemory.stopped.input, header->schemaBytes, &read) != FR_OK ||
        read != header->schemaBytes || memcmp(logIoMemory.stopped.input, logSchemaText, header->schemaBytes)) return false;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceInfo
// 処理概要     全件検証済みのバイナリ識別情報を取得する
// 引数         file: 入力ファイル, header: 識別情報の出力先
// 戻り値       true: 検証済みの現行bin false: 無効な読込状態
/////////////////////////////////////////////////////////////////////
bool logSourceInfo(FIL *file, LogBinaryHeader *header)
{
    if (file == NULL || binarySource.file != file || binarySource.error != FR_OK || header == NULL) return false;
    *header = binarySource.header;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceOpen
// 処理概要     現行binだけを開き全件CRC検証後に解析用途を設定する
// 引数         file: 入力, logNumber: ログ番号, purpose: 読込用途
// 戻り値       FatFs結果
/////////////////////////////////////////////////////////////////////
FRESULT logSourceOpen(FIL *file, int logNumber, LogSourcePurpose purpose)
{
    if (file == NULL || logNumber <= 0 || logNumber > INT16_MAX) return FR_INVALID_PARAMETER;
    uint16_t number = (uint16_t)logNumber;
    if (binarySource.file != NULL || !logStorageCanRead()) return FR_LOCKED;
    if (purpose < LOG_SOURCE_DISTANCE || purpose > LOG_SOURCE_PATH) return FR_INVALID_PARAMETER;
    logReadAheadReset();
    char name[20];
    logBinaryName(name, sizeof(name), number, "bin");
    FRESULT result = f_open(file, name, FA_READ | FA_OPEN_EXISTING);
    if (result != FR_OK) return result;
    LogBinaryHeader header;
    UINT read;
    uint32_t crc;
    if (!logStorageReadContainerHeader(file, number, &header) ||
        f_lseek(file, LOG_BINARY_HEADER_BYTES + (FSIZE_t)header.rows * header.recordBytes + header.schemaBytes) != FR_OK ||
        f_read(file, columnTitle, header.metadataBytes, &read) != FR_OK || read != header.metadataBytes) goto invalid;
    columnTitle[header.metadataBytes] = '\0';
    if (columnTitle[header.metadataBytes - 1U] != '\n' ||
        memchr(columnTitle, '\0', header.metadataBytes) != NULL ||
        ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)columnTitle, header.metadataBytes) != header.metadataCrc ||
        !logStorageScanRecords(file, (uint16_t)header.rows, NULL, NULL, &crc) || crc != header.dataCrc ||
        f_lseek(file, LOG_BINARY_HEADER_BYTES) != FR_OK) goto invalid;
    memset(&binarySource, 0, sizeof(binarySource));
    binarySource.file = file;
    binarySource.header = header;
    binarySource.purpose = purpose;
    logReadAheadReset();
    binarySource.crc = UINT32_MAX;
    clearXYcie();
    return FR_OK;
invalid:
    (void)f_close(file);
    return FR_INVALID_OBJECT;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceClose
// 処理概要     共通読込の状態を解放しファイルを閉じる
// 引数         file: ファイル
// 戻り値       FatFs結果
/////////////////////////////////////////////////////////////////////
FRESULT logSourceClose(FIL *file)
{
    if (binarySource.file != file || file == NULL) return FR_INVALID_OBJECT;
    binarySource.file = NULL;
    logReadAheadReset();
    return f_close(file);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logProcessRecord
// 処理概要     記録を速度補正・曲率直線化・保存方位によるXYへ変換する
// 引数         なし（共通読込状態を使用）
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logProcessRecord(void)
{
    LogRecord *rec = &binarySource.record;
    int16_t speed = (int16_t)rec->encCurrentN;
    if (abs((int32_t)speed - binarySource.previousSpeed) > 500) {
        speed = binarySource.previousSpeed;
        rec->encCurrentN = (uint16_t)speed;
    }
    float dt = (float)(rec->cntlog - binarySource.previousTime) / 1000.0F;
    binarySource.roc = calcROC(speed, rec->gyroVal_Z, dt);
    if (binarySource.purpose == LOG_SOURCE_PATH) {
        calcXYcie((int64_t)rec->encTotalL + rec->encTotalR, rec->imuAngle_Z);
        binarySource.x = xycie.x;
        binarySource.y = xycie.y;
    }
    binarySource.distanceMm += calcDlMm(speed, dt);
    binarySource.previousSpeed = speed;
    binarySource.previousTime = rec->cntlog;
    for (uint16_t i = 0; i < sourceCrossCount; i++) {
        float start = fmaxf(0.0F, cross_start_mm[i] - CROSS_STRAIGHT_MM);
        if (binarySource.distanceMm >= start && binarySource.distanceMm <= cross_end_mm[i] + CROSS_STRAIGHT_MM) {
            binarySource.roc = ROC_STRAIGHT_MAX;
            break;
        }
    }
    if (rec->courseMarker == 3U) binarySource.roc = ROC_STRAIGHT_MAX;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceError
// 処理概要     現行bin読込のエラーを取得する
// 引数         file: ファイル
// 戻り値       0: エラーなし その他: エラー
/////////////////////////////////////////////////////////////////////
int logSourceError(FIL *file) { return file != NULL && binarySource.file == file ? (int)binarySource.error : FR_INVALID_OBJECT; }

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceEof
// 処理概要     メタデータ領域を除いた記録の終端を判定する
// 引数         file: ファイル
// 戻り値       非0: EOF 0: EOF以外
/////////////////////////////////////////////////////////////////////
int logSourceEof(FIL *file)
{
    return file != NULL && binarySource.file == file && binarySource.row == binarySource.header.rows && binarySource.error == FR_OK;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceMetadata
// 処理概要     全CRC検証済みの走行メタデータを取得する
// 引数         file: 読込中のファイル
// 戻り値       メタデータ文字列、無効な状態ではNULL
/////////////////////////////////////////////////////////////////////
const char *logSourceMetadata(FIL *file)
{
    return file != NULL && binarySource.file == file && binarySource.error == FR_OK ? columnTitle : NULL;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceNext
// 処理概要     現行binの次の記録を共通解析行へ復元し終端CRCを確認する
// 引数         file: 読込中のファイル, row: 復元先
// 戻り値       true: 1件取得 false: 終端またはエラー（logSourceErrorで区別）
/////////////////////////////////////////////////////////////////////
bool logSourceNext(FIL *file, LogSourceRow *row)
{
    if (file == NULL || binarySource.file != file) return false;
    if (row == NULL) { binarySource.error = FR_INVALID_PARAMETER; return false; }
    if (binarySource.error != FR_OK) return false;
    if (binarySource.row >= binarySource.header.rows) {
        if (~binarySource.crc != binarySource.header.dataCrc) binarySource.error = FR_INVALID_OBJECT;
        return false;
    }
    uint8_t bytes[LOG_READ_RECORD_BYTES];
    if (binarySource.row == 0U) { LOG_PERF_ADD(fullScans, 1U); }
    FRESULT result = logReadBuffered(file, bytes, LOG_SIZE,
        LOG_BINARY_HEADER_BYTES + (FSIZE_t)binarySource.header.rows * LOG_SIZE);
    if (result != FR_OK) { binarySource.error = result; return false; }
    binarySource.crc = logBinaryCrcUpdate(binarySource.crc, bytes, LOG_SIZE);
    logaddress = bytes;
    logReadRecord(&binarySource.record);
    LOG_PERF_BEGIN(processTick);
    logProcessRecord();
    LOG_PERF_END(processMs, processTick);
    binarySource.row++;
    const LogRecord *r = &binarySource.record;
    *row = (LogSourceRow){
        .cntlog = r->cntlog, .encCurrentN = r->encCurrentN, .gyroValZ = r->gyroVal_Z,
        .courseMarker = r->courseMarker, .encTotalOptimal = (int32_t)r->encTotalOptimal,
        .ROC = binarySource.roc, .targetSpeed = r->targetSpeed, .optimalIndex = r->optimalIndex,
        .slipV2Flags = r->slipV2Flags,
        .x = binarySource.x, .y = binarySource.y
    };
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceValidateSlipRow
// 処理概要     V2判定フラグの定義ビットと解析値の範囲を検証する
// 引数         row: 現行binから復元した行
// 戻り値       true: 正常 false: 不正
/////////////////////////////////////////////////////////////////////
bool logSourceValidateSlipRow(const LogSourceRow *row)
{
	return row != NULL && row->courseMarker >= 0 && row->courseMarker <= (int32_t)UINT8_MAX &&
		row->optimalIndex >= 0 && row->optimalIndex <= INT16_MAX &&
		isfinite(row->ROC) && fabsf(row->ROC) <= INT16_MAX &&
		isfinite(row->targetSpeed) && row->targetSpeed >= 0.0F && row->targetSpeed <= UINT8_MAX &&
		row->slipV2Flags >= 0 &&
		(row->slipV2Flags & ~(int32_t)(SLIP_V2_LONG | SLIP_V2_LAT | SLIP_V2_YAW)) == 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logMetadataReadInt
// 処理概要     走行メタデータから重複のない整数相当値を範囲検証して取得する
// 引数         line: key=valueのメタデータ, key: 項目名, value: 成功時の格納先
// 戻り値       true: 取得成功 false: 欠落・重複・端数・範囲外または不正入力
/////////////////////////////////////////////////////////////////////
bool logMetadataReadInt(const char *line, const char *key, int32_t *value)
{
    if (line == NULL || key == NULL || value == NULL || *key == '\0' ||
        strpbrk(key, ",=\r\n") != NULL) return false;
    const size_t length = strlen(key);
    bool found = false;
    int32_t parsed = 0;
    for (const char *field = line; field != NULL && *field != '\0'; )
    {
        if (strncmp(field, key, length) == 0 && field[length] == '=')
        {
            char *end;
            errno = 0;
            const double number = strtod(field + length + 1U, &end);
            if (found || end == field + length + 1U || errno == ERANGE ||
                (*end != ',' && *end != '\r' && *end != '\n' && *end != '\0') ||
                !isfinite(number) || number < INT32_MIN || number > INT32_MAX ||
                number != trunc(number)) return false;
            parsed = (int32_t)number;
            found = true;
        }
        const char *next = strchr(field, ',');
        field = next != NULL ? next + 1 : NULL;
    }
    if (found) *value = parsed;
    return found;
}
