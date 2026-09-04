#ifndef LOG_SCHEMA_H_
#define LOG_SCHEMA_H_
//====================================//
// インクルード
//====================================//
#include <stdint.h>
#include "SDcard.h"
#include "battery.h"
#include "motor.h"
#include "pathFollower.h"
//====================================//
// シンボル定義
//====================================//
// CSV列の定義を順番どおりに集中管理。
// STOREDはバイナリに保存、DERIVEDはendLogで計算。
// 通常は軽量プロファイルで走行比較に必要な列だけを残す。
// 詳細デバッグ列が必要な場合は、ビルド定義でLOG_SCHEMA_PROFILE_LIGHT=0にする。
// STORED(type, name, fmt, expr) or DERIVED(type, name, fmt, expr)
#ifndef LOG_SCHEMA_PROFILE_LIGHT
#define LOG_SCHEMA_PROFILE_LIGHT 1
#endif

#define LOG_FIELD_LIST_CORE(STORED, DERIVED) \
	STORED(U16, cntlog, "%d", (uint16_t)cntRun) \
	STORED(U16, encCurrentN, "%d", (uint16_t)encCurrentN) \
	STORED(F32, gyroVal_Z, "%f", imuVal.gyro.z) \
	STORED(U8, courseMarker, "%d", courseMarkerLog) \
	STORED(U32, encTotalOptimal, "%d", (uint32_t)encTotalOptimal) \
	DERIVED(F32, ROC, "%f", log_roc) \
	STORED(U8, targetSpeed, "%d", targetSpeed) \
	STORED(U16, optimalIndex, "%d", (uint16_t)optimalIndex) \
	STORED(U8, slipFlag, "%d", (uint8_t)getSlipFlag()) \
	STORED(U8, slipFlagLat, "%d", (uint8_t)getSlipFlagLat()) \
	STORED(S16, lineTraceCtrl, "%d", (int16_t)lineTraceOmegaFBCtrl.pwm) \
	STORED(S16, targetAngularvelo, "%d", (int16_t)log_targetAngularVelocity) \
	STORED(S16, motorpwmL, "%d", (int16_t)motorpwmL) \
	STORED(S16, motorpwmR, "%d", (int16_t)motorpwmR) \
	STORED(U16, batteryVoltage_mV, "%d", (uint16_t)(batteryVoltage_V * 1000.0f)) \
	STORED(S16, motorVoltageCmdL_mV, "%d", (int16_t)(motorVoltageCmdL_V * 1000.0f)) \
	STORED(S16, motorVoltageCmdR_mV, "%d", (int16_t)(motorVoltageCmdR_V * 1000.0f)) \
	STORED(U32, encCurrentCorr_p, "%d", (uint32_t)Control_GetEncCurrentCorr_p()) \
	STORED(F32, linePointX_mm, "%f", pathLogLinePointX_mm) \
	STORED(F32, linePointY_mm, "%f", pathLogLinePointY_mm) \
	STORED(U8, lineValid, "%d", pathLogLineValid) \
	STORED(F32, pathErrorY_mm, "%f", pathLogErrorY_mm) \
	STORED(S16, pathErrorHeading_cdeg, "%d", pathLogErrorHeading_cdeg) \
	STORED(U8, pathState, "%d", pathLogState) \
	STORED(F32, pathLegalMargin_mm, "%f", pathLogLegalMargin_mm) \
	DERIVED(F32, x, "%f", log_x) \
	DERIVED(F32, y, "%f", log_y)

#define LOG_FIELD_LIST_DEBUG(STORED, DERIVED) \
	STORED(F32, acceleVal_X, "%f", imuVal.accele.x) \
	STORED(F32, acceleVal_Y, "%f", imuVal.accele.y) \
	STORED(U8, markerSensor, "%d", (uint8_t)markerSensor) \
	STORED(U8, sgMarkerCount, "%d", (uint8_t)SGmarker) \
	STORED(U32, encRightMarker_p, "%d", (uint32_t)encRightMarker) \
	STORED(U8, patternTrace, "%d", (uint8_t)patternTrace) \
	STORED(F32, slipRatio, "%f", getSlipIndicatorRaw()) \
	STORED(F32, slipRatioLat, "%f", getSlipIndicatorFiltered()) \
	STORED(F32, motorCurrentL, "%f", motorCurrentL) \
	STORED(F32, motorCurrentR, "%f", motorCurrentR) \
	STORED(U8, straightPendingAttempt, "%d", (uint8_t)straightMarkerPendingLog) \
	STORED(U8, lineTraceOmegaFBCtrlkp, "%d", (uint8_t)lineTraceOmegaFBCtrl.kp) \
	STORED(U8, slipLatEnabled, "%d", (uint8_t)getSlipLatEnabled()) \
	STORED(U8, slipLatOnCountEnabled, "%d", (uint8_t)getSlipLatOnCountEnabled()) \
	STORED(U16, slipISumOkCount, "%d", getSlipISumOkCount()) \
	STORED(F32, slipPwmSumF, "%f", getSlipPwmSumF()) \
	STORED(F32, slipISumF, "%f", getSlipISumF()) \
	STORED(F32, slipEncAyF, "%f", getSlipEncAyF()) \
	STORED(F32, slipImuAyF, "%f", getSlipImuAyF())

#if LOG_SCHEMA_PROFILE_LIGHT
#define LOG_FIELD_LIST(STORED, DERIVED) LOG_FIELD_LIST_CORE(STORED, DERIVED)
#else
#define LOG_FIELD_LIST(STORED, DERIVED) \
	LOG_FIELD_LIST_CORE(STORED, DERIVED) \
	LOG_FIELD_LIST_DEBUG(STORED, DERIVED)
#endif
// 詳細デバッグ列の追加候補:
// STORED(F32, slipDistScaleF, "%f", Control_GetSlipDistScale())
// STORED(U32, distEncRaw_p, "%d", (uint32_t)Control_GetDistEncRaw_p())
// STORED(U32, distCorr_p, "%d", (uint32_t)Control_GetDistCorr_p())
// STORED(U32, distSlipLoss_p, "%d", (uint32_t)Control_GetDistSlipLoss_p())
// バイナリログで保存する型の対応表。
#define LOG_CTYPE_U8  uint8_t
#define LOG_CTYPE_U16 uint16_t
#define LOG_CTYPE_S16 int16_t
#define LOG_CTYPE_U32 uint32_t
#define LOG_CTYPE_F32 float

#define LOG_FIELD_SIZE_U8  1U
#define LOG_FIELD_SIZE_U16 2U
#define LOG_FIELD_SIZE_S16 2U
#define LOG_FIELD_SIZE_U32 4U
#define LOG_FIELD_SIZE_F32 4U

#define LOG_RECORD_SIZE_ADD(type, name, fmt, expr) + LOG_FIELD_SIZE_##type
#define LOG_RECORD_SIZE_SKIP(type, name, fmt, expr)
// 1レコード分のバイトサイズ。
enum { LOG_RECORD_SIZE_BYTES = 0 LOG_FIELD_LIST(LOG_RECORD_SIZE_ADD, LOG_RECORD_SIZE_SKIP) };

#endif // LOG_SCHEMA_H_
