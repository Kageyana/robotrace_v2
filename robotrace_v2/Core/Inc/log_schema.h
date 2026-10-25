#ifndef LOG_SCHEMA_H_
#define LOG_SCHEMA_H_
//====================================//
// インクルード
//====================================//
#include <stdint.h>
#include "slipDetection.h"
#include "SDcard.h"
#include "battery.h"
#include "motor.h"
#include "pathFollower.h"
//====================================//
// シンボル定義
//====================================//
// CSV列の定義を順番どおりに集中管理。
// STOREDはバイナリに保存、DERIVEDはCSV変換時に計算。RLOG v2の列定義もここから生成する。
// 通常は軽量プロファイルで走行比較に必要な列だけを残す。
// 詳細デバッグ列が必要な場合は、ビルド定義でLOG_SCHEMA_PROFILE_LIGHT=0にする。
// STORED(type, name, fmt, expr) or DERIVED(type, name, fmt, expr)
#ifndef LOG_SCHEMA_PROFILE_LIGHT
#define LOG_SCHEMA_PROFILE_LIGHT 1
#endif

#define LOG_FIELD_LIST_CORE(STORED, DERIVED) \
	STORED(U16, cntlog, "%d", (uint16_t)cntRun) \
	STORED(U16, encCurrentN, "%d", (uint16_t)encCurrentN) \
	STORED(S16, encCurrentL, "%d", encCurrentL) \
	STORED(S16, encCurrentR, "%d", encCurrentR) \
	STORED(S32, encTotalL, "%ld", (int32_t)encTotalL) \
	STORED(S32, encTotalR, "%ld", (int32_t)encTotalR) \
	STORED(F32, gyroVal_Z, "%f", imuVal.gyro.z) \
	STORED(F32, acceleVal_X, "%f", imuVal.accele.x) \
	STORED(F32, acceleVal_Y, "%f", imuVal.accele.y) \
	STORED(F32, acceleVal_Z, "%f", imuVal.accele.z) \
	STORED(U8, courseMarker, "%d", courseMarkerLog) \
	STORED(U32, encTotalOptimal, "%d", (uint32_t)encTotalOptimal) \
	DERIVED(F32, ROC, "%f", log_roc) \
	STORED(U8, targetSpeed, "%d", targetSpeed) \
	STORED(U16, optimalIndex, "%d", (uint16_t)optimalIndex) \
	STORED(S16, lineTraceCtrl, "%d", (int16_t)lineTraceOmegaFBCtrl.pwm) \
	STORED(S16, targetAngularvelo, "%d", (int16_t)log_targetAngularVelocity) \
	STORED(U16, batteryVoltage_mV, "%d", (uint16_t)(batteryVoltage_V * 1000.0f)) \
	STORED(S16, motorVoltageCmdL_mV, "%d", (int16_t)(motorVoltageCmdL_V * 1000.0f)) \
	STORED(S16, motorVoltageCmdR_mV, "%d", (int16_t)(motorVoltageCmdR_V * 1000.0f)) \
	STORED(U32, encCurrentCorr_p, "%d", (uint32_t)Control_GetEncCurrentCorr_p()) \
	STORED(U16, lSensorCari0, "%d", lSensorCari[0]) \
	STORED(U16, lSensorCari1, "%d", lSensorCari[1]) \
	STORED(U16, lSensorCari2, "%d", lSensorCari[2]) \
	STORED(U16, lSensorCari3, "%d", lSensorCari[3]) \
	STORED(U16, lSensorCari4, "%d", lSensorCari[4]) \
	STORED(U16, lSensorCari5, "%d", lSensorCari[5]) \
	STORED(U16, lSensorCari6, "%d", lSensorCari[6]) \
	STORED(U16, lSensorCari7, "%d", lSensorCari[7]) \
	STORED(U16, lSensorCari8, "%d", lSensorCari[8]) \
	STORED(U16, lSensorCari9, "%d", lSensorCari[9]) \
	DERIVED(F32, x, "%f", log_x) \
	DERIVED(F32, y, "%f", log_y)

#define LOG_FIELD_LIST_DEBUG(STORED, DERIVED) \
	STORED(U8, markerSensor, "%d", (uint8_t)markerSensor) \
	STORED(U8, sgMarkerCount, "%d", (uint8_t)SGmarker) \
	STORED(U32, encRightMarker_p, "%d", (uint32_t)encRightMarker) \
	STORED(U8, patternTrace, "%d", (uint8_t)patternTrace) \
	STORED(F32, motorCurrentL, "%f", motorCurrentL) \
	STORED(F32, motorCurrentR, "%f", motorCurrentR) \
	STORED(U8, straightPendingAttempt, "%d", (uint8_t)straightMarkerPendingLog) \
	STORED(U8, lineTraceOmegaFBCtrlkp, "%d", (uint8_t)lineTraceOmegaFBCtrl.kp)

// 通常ログの既存列順を保つため末尾へ追加する。詳細ログでは既存の電流列を使用する。
// 電流[A]はADC移動平均・ゼロ点校正後の最新値を保存する。
#define LOG_FIELD_LIST_CURRENT(STORED, DERIVED) \
	STORED(F32, motorCurrentL, "%f", motorCurrentL) \
	STORED(F32, motorCurrentR, "%f", motorCurrentR)

// 既存列の位置を維持し、通常・詳細とも末尾にIMU診断列を追加する。
// 温度は最新取得値[°C]（5ms周期更新）、無効時は-999。角度[deg]はXYが加速度融合、Zがジャイロ積分。
#define LOG_FIELD_LIST_IMU(STORED, DERIVED) \
	STORED(F32, imuTemp_C, "%f", (imuVal.tempValid ? imuVal.temp : BMI088_TEMP_INVALID_C)) \
	STORED(F32, gyroVal_X, "%f", imuVal.gyro.x) \
	STORED(F32, gyroVal_Y, "%f", imuVal.gyro.y) \
	STORED(F32, imuAngle_X, "%f", imuVal.angle.x) \
	STORED(F32, imuAngle_Y, "%f", imuVal.angle.y) \
	STORED(F32, imuAngle_Z, "%f", imuVal.angle.z)

#if LOG_SCHEMA_PROFILE_LIGHT
#define LOG_BASE_FIELD_LIST(STORED, DERIVED) \
	LOG_FIELD_LIST_CORE(STORED, DERIVED) \
	LOG_FIELD_LIST_IMU(STORED, DERIVED) \
	LOG_FIELD_LIST_CURRENT(STORED, DERIVED)
#else
#define LOG_BASE_FIELD_LIST(STORED, DERIVED) \
	LOG_FIELD_LIST_CORE(STORED, DERIVED) \
	LOG_FIELD_LIST_DEBUG(STORED, DERIVED) \
	LOG_FIELD_LIST_IMU(STORED, DERIVED)
#endif
// V2の瞬時残差・判定フラグと補助診断を通常・詳細の両方へ保存する。
#define LOG_FIELD_LIST_V2(STORED, DERIVED) \
    STORED(F32, slipV2LongFast, "%f", slipV2.result.longFast) \
    STORED(F32, slipV2LongSlow, "%f", slipV2.result.longSlow) \
    STORED(F32, slipV2Lat, "%f", slipV2.result.lateral) \
    STORED(F32, slipV2YawDps, "%f", slipV2.result.yawDps) \
    STORED(U8, slipV2Flags, "%d", slipV2.result.flags) \
    STORED(U16, slipV2CurrentHighSamples, "%d", slipV2.result.currentHighSamples) \
    STORED(U16, slipV2PwmSum, "%d", slipV2.result.pwmSum)

// 横補正に使用した瞬時値を保存する。有効性・無効理由・区間統計は保存しない。
#define LOG_FIELD_LIST_LAT_DIAGNOSTICS(STORED, DERIVED) \
    STORED(F32, imuYawAlphaFiltered, "%f", slipV2.lateralCandidate.result.alphaRadps2) \
    STORED(F32, imuLatTangentialApplied, "%f", slipV2.lateralCandidate.result.tangentialAppliedXMps2)

// PATH/SHORTCUTの5ms周期診断。経路モード以外は0を保存する。
// 方位偏差は最近傍経路接線に対する差[0.01deg]、操舵3成分は符号付きの飽和前指令[deg/s]。
#define LOG_FIELD_LIST_PATH(STORED, DERIVED) \
    STORED(F32, pathErrorY_mm, "%f", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogErrorY_mm : 0.0f)) \
    STORED(S16, pathErrorHeading_cdeg, "%d", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogErrorHeading_cdeg : 0)) \
    STORED(U8, pathState, "%d", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogState : PATH_STATE_INACTIVE)) \
    STORED(F32, pathYawFF_dps, "%f", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogYawFF_dps : 0.0f)) \
    STORED(F32, pathYawLateralFB_dps, "%f", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogYawLateralFB_dps : 0.0f)) \
    STORED(F32, pathYawHeadingFB_dps, "%f", ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) ? pathLogYawHeadingFB_dps : 0.0f))

// 通常56列・154 B、詳細62列・163 B。既存列の末尾へ経路診断6列を追加する。
#define LOG_FIELD_LIST(STORED, DERIVED) \
    LOG_BASE_FIELD_LIST(STORED, DERIVED) \
    LOG_FIELD_LIST_V2(STORED, DERIVED) \
    LOG_FIELD_LIST_LAT_DIAGNOSTICS(STORED, DERIVED) \
    LOG_FIELD_LIST_PATH(STORED, DERIVED)

// バイナリログで保存する型の対応表。
#define LOG_CTYPE_U8  uint8_t
#define LOG_CTYPE_U16 uint16_t
#define LOG_CTYPE_S16 int16_t
#define LOG_CTYPE_U32 uint32_t
#define LOG_CTYPE_S32 int32_t
#define LOG_CTYPE_F32 float

#define LOG_FIELD_SIZE_U8  1U
#define LOG_FIELD_SIZE_U16 2U
#define LOG_FIELD_SIZE_S16 2U
#define LOG_FIELD_SIZE_U32 4U
#define LOG_FIELD_SIZE_S32 4U
#define LOG_FIELD_SIZE_F32 4U

#define LOG_RECORD_SIZE_ADD(type, name, fmt, expr) + LOG_FIELD_SIZE_##type
#define LOG_RECORD_SIZE_SKIP(type, name, fmt, expr)
// 1レコード分のバイトサイズ。
enum { LOG_RECORD_SIZE_BYTES = 0 LOG_FIELD_LIST(LOG_RECORD_SIZE_ADD, LOG_RECORD_SIZE_SKIP) };

//====================================//
// グローバル変数の宣言、定義
//====================================//

//====================================//
// プロトタイプ宣言
//====================================//

#endif // LOG_SCHEMA_H_
