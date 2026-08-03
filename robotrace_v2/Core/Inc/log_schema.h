#ifndef LOG_SCHEMA_H_
#define LOG_SCHEMA_H_
//====================================//
// インクルード
//====================================//
#include <stdint.h>
#include "SDcard.h"
#include "timer.h"
#include "encoder.h"
//====================================//
// シンボル定義
//====================================//
// CSV列の定義を順番どおりに集中管理。
// STOREDはバイナリに保存、DERIVEDはendLogで計算。
// このリストを編集して、CSVログフィールドを追加または削除してください。
// STORED(type, name, fmt, expr) or DERIVED(type, name, fmt, expr)
#ifndef LOG_SCHEMA_PROFILE_LIGHT
#define LOG_SCHEMA_PROFILE_LIGHT 1
#endif

#define LOG_FIELD_LIST_FULL(STORED, DERIVED) \
	STORED(U16, cntlog, "%d", (uint16_t)cntRun) \
	STORED(U16, encCurrentN, "%d", (uint16_t)encCurrentN) \
	STORED(F32, gyroVal_Z, "%f", imuVal.gyro.z) \
	STORED(U8, courseMarker, "%d", courseMarkerLog) \
	STORED(U32, encTotalOptimal, "%d", (uint32_t)encTotalOptimal) \
	DERIVED(F32, ROC, "%f", log_roc) \
	STORED(S16, encCurrentL, "%d", encCurrentL) \
	STORED(S16, encCurrentR, "%d", encCurrentR) \
	STORED(U8, targetSpeed, "%d", targetSpeed) \
	STORED(S16, targetSpeedL, "%d", targetSpeedLLog) \
	STORED(S16, targetSpeedR, "%d", targetSpeedRLog) \
	STORED(U8, speedTargetClip, "%d", speedTargetClipLog) \
	STORED(U16, optimalIndex, "%d", (uint16_t)optimalIndex) \
	STORED(U8, slipFlag, "%d", (uint8_t)getSlipFlag()) \
	STORED(U8, slipFlagLat, "%d", (uint8_t)getSlipFlagLat()) \
	STORED(S16, lineTraceCtrl, "%d", (int16_t)lineTraceOmegaFBCtrl.pwm) \
	STORED(S16, targetAngularvelo, "%d", (int16_t)log_targetAngularVelocity) \
	STORED(S16, motorpwmL, "%d", (int16_t)motorpwmL) \
	STORED(S16, motorpwmR, "%d", (int16_t)motorpwmR) \
	STORED(F32, acceleVal_X, "%f", imuVal.accele.x) \
	STORED(F32, acceleVal_Y, "%f", imuVal.accele.y) \
	STORED(F32, slipLongResidual_mps2, "%f", getSlipIndicatorRaw()) \
	STORED(F32, slipLatResidual_mps2, "%f", getSlipIndicatorFiltered()) \
	STORED(F32, motorCurrentL, "%f", motorCurrentL) \
	STORED(F32, motorCurrentR, "%f", motorCurrentR) \
	STORED(U32, encCurrentCorr_p, "%d", (uint32_t)Control_GetEncCurrentCorr_p()) \
	STORED(U8, straightPendingAttempt, "%d", (uint8_t)straightMarkerPendingLog) \
	STORED(U8, lineTraceOmegaFBCtrlkp, "%d", (uint8_t)lineTraceOmegaFBCtrl.kp) \
	STORED(U8, lineTraceOmegaFBCtrlkd, "%d", (uint8_t)lineTraceOmegaFBCtrl.kd) \
	DERIVED(F32, x, "%f", log_x) \
	DERIVED(F32, y, "%f", log_y)

#define LOG_FIELD_LIST_LIGHT(STORED, DERIVED) \
	STORED(U16, cntlog, "%d", (uint16_t)cntRun) \
	STORED(U16, encCurrentN, "%d", (uint16_t)encCurrentN) \
	STORED(F32, gyroVal_Z, "%f", imuVal.gyro.z) \
	STORED(U8, courseMarker, "%d", courseMarkerLog) \
	STORED(U32, encTotalOptimal, "%d", (uint32_t)encTotalOptimal) \
	DERIVED(F32, ROC, "%f", log_roc) \
	STORED(U8, targetSpeed, "%d", targetSpeed) \
	STORED(U16, logPadU16_01, "%d", 0) \
	STORED(U8, logPadU8_01, "%d", 0) \
	STORED(U16, optimalIndex, "%d", (uint16_t)optimalIndex) \
	STORED(U8, slipFlag, "%d", (uint8_t)getSlipFlag()) \
	STORED(U8, slipFlagLat, "%d", (uint8_t)getSlipFlagLat()) \
	STORED(S16, lineTraceCtrl, "%d", (int16_t)lineTraceOmegaFBCtrl.pwm) \
	STORED(S16, motorpwmL, "%d", (int16_t)motorpwmL) \
	STORED(S16, motorpwmR, "%d", (int16_t)motorpwmR) \
	STORED(U32, encCurrentCorr_p, "%d", (uint32_t)Control_GetEncCurrentCorr_p()) \
	STORED(U8, lineTraceOmegaFBCtrlkp, "%d", (uint8_t)lineTraceOmegaFBCtrl.kp) \
	STORED(U8, lineTraceOmegaFBCtrlkd, "%d", (uint8_t)lineTraceOmegaFBCtrl.kd) \
	STORED(U16, logPadU16_00, "%d", 0) \
	STORED(U8, logPadU8_00, "%d", 0) \
	DERIVED(F32, x, "%f", log_x) \
	DERIVED(F32, y, "%f", log_y)

#if LOG_SCHEMA_PROFILE_LIGHT
#define LOG_FIELD_LIST(STORED, DERIVED) LOG_FIELD_LIST_LIGHT(STORED, DERIVED)
#else
#define LOG_FIELD_LIST(STORED, DERIVED) LOG_FIELD_LIST_FULL(STORED, DERIVED)
#endif


	/* STORED(F32, slipDistScaleF, "%f", Control_GetSlipDistScale()) */
	/* STORED(U32, distEncRaw_p, "%d", (uint32_t)Control_GetDistEncRaw_p()) */
	/* STORED(U32, distCorr_p, "%d", (uint32_t)Control_GetDistCorr_p()) */
	/* STORED(U32, distSlipLoss_p, "%d", (uint32_t)Control_GetDistSlipLoss_p()) */
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
