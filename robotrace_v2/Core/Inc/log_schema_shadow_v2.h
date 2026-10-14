#ifndef LOG_SCHEMA_SHADOW_V2_H_
#define LOG_SCHEMA_SHADOW_V2_H_
// 初回SHADOWの62列/157B・77列/194Bを独立して固定する。
#define LOG_V2_FIELD_LIST_CORE(STORED, DERIVED) \
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
	STORED(U8, slipFlag, "%d", (uint8_t)getSlipFlag()) \
	STORED(U8, slipFlagLat, "%d", (uint8_t)getSlipFlagLat()) \
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

#define LOG_V2_FIELD_LIST_DEBUG(STORED, DERIVED) \
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

// 通常ログの既存列順を保つため末尾へ追加する。詳細ログでは既存の電流列を使用する。
// 電流[A]はADC移動平均・ゼロ点校正後の最新値を保存する。
#define LOG_V2_FIELD_LIST_CURRENT(STORED, DERIVED) \
	STORED(F32, motorCurrentL, "%f", motorCurrentL) \
	STORED(F32, motorCurrentR, "%f", motorCurrentR)

// 既存列の位置を維持し、通常・詳細とも末尾にIMU診断列を追加する。
// 温度は最新取得値[°C]（5ms周期更新）、無効時は-999。角度[deg]はXYが加速度融合、Zがジャイロ積分。
#define LOG_V2_FIELD_LIST_IMU(STORED, DERIVED) \
	STORED(F32, imuTemp_C, "%f", (imuVal.tempValid ? imuVal.temp : BMI088_TEMP_INVALID_C)) \
	STORED(F32, gyroVal_X, "%f", imuVal.gyro.x) \
	STORED(F32, gyroVal_Y, "%f", imuVal.gyro.y) \
	STORED(F32, imuAngle_X, "%f", imuVal.angle.x) \
	STORED(F32, imuAngle_Y, "%f", imuVal.angle.y) \
	STORED(F32, imuAngle_Z, "%f", imuVal.angle.z)

// 新旧判定を混同しないため、SHADOW診断列を末尾に追加する。
#define LOG_V2_FIELD_LIST_SHADOW(STORED, DERIVED) \
    STORED(F32, slipV2LongFast, "%f", slipShadow.result.longFast) \
    STORED(F32, slipV2LongSlow, "%f", slipShadow.result.longSlow) \
    STORED(F32, slipV2Lat, "%f", slipShadow.result.lateral) \
    STORED(F32, slipV2YawDps, "%f", slipShadow.result.yawDps) \
    STORED(U8, slipV2Flags, "%d", slipShadow.result.flags) \
    STORED(U8, slipV2Valid, "%d", slipShadow.result.valid) \
    STORED(U16, slipV2Reason, "%d", slipShadow.result.reason) \
    STORED(U8, slipV2CurrentValid, "%d", slipShadow.result.currentValid) \
    STORED(U16, slipV2CurrentHighSamples, "%d", slipShadow.result.currentHighSamples) \
    STORED(U16, slipV2PwmSum, "%d", slipShadow.result.pwmSum) \
    STORED(F32, slipV2PeakLong, "%f", slipShadow.interval.peakLong) \
    STORED(F32, slipV2PeakLat, "%f", slipShadow.interval.peakLat) \
    STORED(F32, slipV2PeakYawDps, "%f", slipShadow.interval.peakYawDps) \
    STORED(U16, slipV2OnLongMs, "%d", slipShadow.interval.onLongMs) \
    STORED(U16, slipV2OnLatMs, "%d", slipShadow.interval.onLatMs) \
    STORED(U16, slipV2OnYawMs, "%d", slipShadow.interval.onYawMs) \
    STORED(U16, slipV2ValidMs, "%d", slipShadow.interval.validMs) \
    STORED(U16, slipV2TotalMs, "%d", slipShadow.interval.totalMs) \
    STORED(U16, slipV2IntervalReason, "%d", slipShadow.interval.invalidReason)

#define LOG_V2_FIELD_LIST_LIGHT(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_CORE(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_IMU(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_CURRENT(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_SHADOW(STORED, DERIVED)
#define LOG_V2_FIELD_LIST_DEBUG_PROFILE(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_CORE(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_DEBUG(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_IMU(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_SHADOW(STORED, DERIVED)
#endif
