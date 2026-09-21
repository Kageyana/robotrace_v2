#ifndef RUN_GUARD_H_
#define RUN_GUARD_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_AdcSampleUsable
// 処理概要     ADC開始時と完了時のLED位相が一致するか判定する
// 引数         active:変換中、latched:開始時位相、current:現在位相
// 戻り値       true:サンプルを採用できる
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_AdcSampleUsable(bool active, uint8_t latched, uint8_t current)
{
	return active && latched == current;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_LineFresh
// 処理概要     完成したライン観測が3制御周期未満か判定する
// 引数         active:走行計測中、seen:観測済み、tick:現在周期、last:最終更新周期
// 戻り値       true:新鮮または計測外
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_LineFresh(bool active, bool seen, uint32_t tick, uint32_t last)
{
	return !active || (seen && (uint32_t)(tick - last) < 3U);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_ImuReadUsable
// 処理概要     ジャイロと加速度が共に取得できたか判定する
// 引数         gyroOk:ジャイロ成功、accelOk:加速度成功
// 戻り値       true:制御へ渡せる
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_ImuReadUsable(bool gyroOk, bool accelOk)
{
	return gyroOk && accelOk;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_PrimaryImuCalibrated
// 処理概要     一次経路に必要な走行前IMU校正を判定する
// 引数         ready:校正完了、samples:採用数、errors:読出し異常数、expected:必要数
// 戻り値       true:必要数を異常なしで採用
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_PrimaryImuCalibrated(bool ready, uint16_t samples,
	uint16_t errors, uint16_t expected)
{
	return ready && expected > 0U && samples == expected && errors == 0U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_CanAdvanceAutoRun
// 処理概要     ログ保存と一次走行の閉路検証を確認する
// 引数         saved:CSV保存完了、primary:一次走行、closureValid:経路有効
// 戻り値       true:次走へ進んでよい
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_CanAdvanceAutoRun(bool saved, bool primary, bool closureValid)
{
	return saved && (!primary || closureValid);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_CompleteCsvRow
// 処理概要     CSV行が改行まで保存されたか確認する
// 引数         line: CSV行文字列
// 戻り値       true:改行終端あり
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_CompleteCsvRow(const char *line)
{
	return line != NULL && (strchr(line, '\n') != NULL || strchr(line, '\r') != NULL);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 RunGuard_CsvRowsMatch
// 処理概要     新形式の期待行数と実読取行数を照合する
// 引数         expected:ヘッダの行数(0は旧形式)、actual:実読取行数
// 戻り値       true:一致または旧形式
/////////////////////////////////////////////////////////////////////
static inline bool RunGuard_CsvRowsMatch(uint32_t expected, uint32_t actual)
{
	return expected == 0U || expected == actual;
}

#endif // RUN_GUARD_H_
