#include <assert.h>
#include "../../robotrace_v2/Core/Inc/runGuard.h"

int main(void)
{
	// ADC開始・完了間で位相が変わった場合と未開始コールバックは破棄。
	assert(RunGuard_AdcSampleUsable(true, 0U, 0U));
	assert(RunGuard_AdcSampleUsable(true, 1U, 1U));
	assert(!RunGuard_AdcSampleUsable(true, 0U, 1U));
	assert(!RunGuard_AdcSampleUsable(false, 1U, 1U));

	// 更新から3周期目は古い観測。周期カウンタの折り返しも許容。
	assert(RunGuard_LineFresh(true, true, 12U, 10U));
	assert(!RunGuard_LineFresh(true, true, 13U, 10U));
	assert(!RunGuard_LineFresh(true, false, 1U, 0U));
	assert(RunGuard_LineFresh(true, true, 1U, UINT32_MAX));

	// IMU失敗値は使用不可、偽ゴール・SD欠落でautoStartは進まない。
	assert(RunGuard_ImuReadUsable(true, true));
	assert(!RunGuard_ImuReadUsable(true, false));
	assert(!RunGuard_ImuReadUsable(false, true));
	assert(RunGuard_PrimaryImuCalibrated(true, 100U, 0U, 100U));
	assert(!RunGuard_PrimaryImuCalibrated(false, 100U, 0U, 100U));
	assert(!RunGuard_PrimaryImuCalibrated(true, 99U, 0U, 100U));
	assert(!RunGuard_PrimaryImuCalibrated(true, 100U, 1U, 100U));
	assert(!RunGuard_CanAdvanceAutoRun(true, true, false));
	assert(!RunGuard_CanAdvanceAutoRun(false, true, true));
	assert(RunGuard_CanAdvanceAutoRun(true, true, true));
	assert(RunGuard_CanAdvanceAutoRun(true, false, false));
	assert(RunGuard_CompleteCsvRow("1,2,3\n"));
	assert(!RunGuard_CompleteCsvRow("1,2,3"));
	assert(RunGuard_CsvRowsMatch(12U, 12U));
	assert(!RunGuard_CsvRowsMatch(12U, 11U));
	assert(RunGuard_CsvRowsMatch(0U, 12U));
	return 0;
}
