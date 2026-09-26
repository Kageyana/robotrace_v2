#include "autoRun.h"

#include <stdio.h>
#include <string.h>

AutoRunState autoRunState = {0};

static const AutoRunMode defaultModes[AUTO_RUN_CONFIG_SLOT_COUNT] = {
	AUTO_RUN_MODE_DISTANCE,
	AUTO_RUN_MODE_SLIP,
	AUTO_RUN_MODE_PATH,
	AUTO_RUN_MODE_SHORTCUT
};

/////////////////////////////////////////////////////////////////////
// モジュール名 resetSeriesState
// 処理概要     設定を維持したままオートスタート系列の実行状態を初期化する
// 引数         state: オートスタート状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void resetSeriesState(AutoRunState *state)
{
	state->seriesActive = false;
	state->currentPlanValid = false;
	state->currentRunNumber = 0U;
	state->previousMode = AUTO_RUN_MODE_INVALID;
	state->currentMode = AUTO_RUN_MODE_INVALID;
	state->firstLogNumber = 0;
	state->previousLogNumber = 0;
	state->slipSourceLogNumber = 0;
	state->currentPrimaryLogNumber = 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunSetDefaults
// 処理概要     オートスタート2～5走目の既定方式を設定する
// 引数         modes: 既定方式の格納先
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void autoRunSetDefaults(AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT])
{
	if (modes == NULL)
	{
		return;
	}
	memcpy(modes, defaultModes, sizeof(defaultModes));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 parseMode
// 処理概要     方式名を列挙値へ変換する
// 引数         text: 方式名, length: 文字数, mode: 変換結果
// 戻り値       true: 変換成功 false: 未対応の方式名
/////////////////////////////////////////////////////////////////////
static bool parseMode(const char *text, size_t length, AutoRunMode *mode)
{
	if (length == 8U && memcmp(text, "DISTANCE", 8U) == 0)
	{
		*mode = AUTO_RUN_MODE_DISTANCE;
		return true;
	}
	if (length == 4U && memcmp(text, "SLIP", 4U) == 0)
	{
		*mode = AUTO_RUN_MODE_SLIP;
		return true;
	}
	if (length == 4U && memcmp(text, "PATH", 4U) == 0)
	{
		*mode = AUTO_RUN_MODE_PATH;
		return true;
	}
	if (length == 8U && memcmp(text, "SHORTCUT", 8U) == 0)
	{
		*mode = AUTO_RUN_MODE_SHORTCUT;
		return true;
	}
	return false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 trimField
// 処理概要     CSVフィールド両端の空白とタブを除外する
// 引数         start: 開始位置, end: 終端位置
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void trimField(const char **start, const char **end)
{
	while (*start < *end && (**start == ' ' || **start == '\t'))
	{
		(*start)++;
	}
	while (*end > *start && ((*end)[-1] == ' ' || (*end)[-1] == '\t'))
	{
		(*end)--;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunParseConfig
// 処理概要     オートスタート方式設定を検証して走行番号ごとに割り当てる
// 引数         text: 設定ファイル本文, modes: 走行2～5の方式格納先
// 戻り値       true: 全設定が有効 false: 欠落・不正・依存関係違反
/////////////////////////////////////////////////////////////////////
bool autoRunParseConfig(const char *text, AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT])
{
	AutoRunMode parsed[AUTO_RUN_CONFIG_SLOT_COUNT] = {AUTO_RUN_MODE_INVALID};
	bool seen[AUTO_RUN_CONFIG_SLOT_COUNT] = {false};
	const char *line;

	if (text == NULL || modes == NULL)
	{
		return false;
	}

	line = text;
	while (*line != '\0')
	{
		const char *lineEnd = line;
		const char *comma;
		const char *runStart;
		const char *runEnd;
		const char *modeStart;
		const char *modeEnd;
		unsigned long runNumber;
		AutoRunMode mode;

		while (*lineEnd != '\0' && *lineEnd != '\n' && *lineEnd != '\r')
		{
			lineEnd++;
		}
		if (lineEnd == line)
		{
			return false;
		}

		comma = memchr(line, ',', (size_t)(lineEnd - line));
		if (comma == NULL || memchr(comma + 1, ',', (size_t)(lineEnd - comma - 1)) != NULL)
		{
			return false;
		}
		runStart = line;
		runEnd = comma;
		modeStart = comma + 1;
		modeEnd = lineEnd;
		trimField(&runStart, &runEnd);
		trimField(&modeStart, &modeEnd);
		if (runStart == runEnd || (runEnd - runStart) != 1 || *runStart < '2' || *runStart > '5')
		{
			return false;
		}
		runNumber = (unsigned long)(*runStart - '0');
		if (!parseMode(modeStart, (size_t)(modeEnd - modeStart), &mode))
		{
			return false;
		}

		unsigned int slot = (unsigned int)runNumber - 2U;
		if (seen[slot])
		{
			return false;
		}
		seen[slot] = true;
		parsed[slot] = mode;

		if (*lineEnd == '\r' && lineEnd[1] == '\n')
		{
			line = lineEnd + 2;
		}
		else if (*lineEnd != '\0')
		{
			line = lineEnd + 1;
		}
		else
		{
			line = lineEnd;
		}
	}

	for (unsigned int slot = 0U; slot < AUTO_RUN_CONFIG_SLOT_COUNT; slot++)
	{
		if (!seen[slot])
		{
			return false;
		}
	}
	for (unsigned int slot = 0U; slot < AUTO_RUN_CONFIG_SLOT_COUNT; slot++)
	{
		if (parsed[slot] == AUTO_RUN_MODE_SLIP &&
			(slot == 0U || parsed[slot - 1U] != AUTO_RUN_MODE_DISTANCE))
		{
			return false;
		}
	}
	memcpy(modes, parsed, sizeof(parsed));
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunFormatConfig
// 処理概要     オートスタート方式設定を走行番号順のCSV文字列にする
// 引数         modes: 方式配列, buffer: 出力先, capacity: 出力容量, length: 出力長
// 戻り値       true: 変換成功 false: 方式または容量が不正
/////////////////////////////////////////////////////////////////////
bool autoRunFormatConfig(const AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT], char *buffer, size_t capacity, size_t *length)
{
	static const char *modeNames[AUTO_RUN_CONFIG_SLOT_COUNT] = {"DISTANCE", "SLIP", "PATH", "SHORTCUT"};
	char text[AUTO_RUN_CONFIG_TEXT_CAPACITY];
	size_t used = 0U;

	if (modes == NULL || buffer == NULL || length == NULL)
	{
		return false;
	}
	for (unsigned int slot = 0U; slot < AUTO_RUN_CONFIG_SLOT_COUNT; slot++)
	{
		const char *name = NULL;
		switch (modes[slot])
		{
		case AUTO_RUN_MODE_DISTANCE: name = modeNames[0]; break;
		case AUTO_RUN_MODE_SLIP: name = modeNames[1]; break;
		case AUTO_RUN_MODE_PATH: name = modeNames[2]; break;
		case AUTO_RUN_MODE_SHORTCUT: name = modeNames[3]; break;
		default: return false;
		}
		if (modes[slot] == AUTO_RUN_MODE_SLIP &&
			(slot == 0U || modes[slot - 1U] != AUTO_RUN_MODE_DISTANCE))
		{
			return false;
		}
		int written = snprintf(text + used, sizeof(text) - used, "%u,%s\n", slot + 2U, name);
		if (written < 0 || (size_t)written >= sizeof(text) - used)
		{
			return false;
		}
		used += (size_t)written;
	}
	if (capacity <= used)
	{
		return false;
	}
	memcpy(buffer, text, used + 1U);
	*length = used;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunLoadConfig
// 処理概要     設定を読み込み、不正・欠落時は既定値でファイル全体を修復する
// 引数         state: オートスタート状態, readConfig: 読込関数, writeConfig: 書込関数, context: I/O状態
// 戻り値       設定利用可能、修復失敗、またはSD読込エラー
/////////////////////////////////////////////////////////////////////
AutoRunConfigLoadResult autoRunLoadConfig(AutoRunState *state, AutoRunConfigRead readConfig, AutoRunConfigWrite writeConfig, void *context)
{
	char text[AUTO_RUN_CONFIG_TEXT_CAPACITY];
	AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT];
	size_t length = 0U;
	AutoRunConfigReadResult readResult;

	if (state == NULL)
	{
		return AUTO_RUN_CONFIG_LOAD_IO_ERROR;
	}
	state->configReady = false;
	readResult = (readConfig != NULL) ? readConfig(context, text, sizeof(text), &length) : AUTO_RUN_CONFIG_READ_IO_ERROR;
	if (readResult == AUTO_RUN_CONFIG_READ_IO_ERROR)
	{
		resetSeriesState(state);
		return AUTO_RUN_CONFIG_LOAD_IO_ERROR;
	}
	if (readResult != AUTO_RUN_CONFIG_READ_MISSING && readResult != AUTO_RUN_CONFIG_READ_INVALID &&
		readResult != AUTO_RUN_CONFIG_READ_SUCCESS)
	{
		resetSeriesState(state);
		return AUTO_RUN_CONFIG_LOAD_IO_ERROR;
	}
	if (readResult == AUTO_RUN_CONFIG_READ_SUCCESS)
	{
		if (length >= sizeof(text))
		{
			readResult = AUTO_RUN_CONFIG_READ_INVALID;
		}
		else if (memchr(text, '\0', length) != NULL)
		{
			readResult = AUTO_RUN_CONFIG_READ_INVALID;
		}
		else
		{
			text[length] = '\0';
			if (autoRunParseConfig(text, modes))
			{
				memcpy(state->modeByRun, modes, sizeof(modes));
				state->configReady = true;
				resetSeriesState(state);
				return AUTO_RUN_CONFIG_LOAD_READY;
			}
			readResult = AUTO_RUN_CONFIG_READ_INVALID;
		}
	}
	if (readResult == AUTO_RUN_CONFIG_READ_IO_ERROR)
	{
		resetSeriesState(state);
		return AUTO_RUN_CONFIG_LOAD_IO_ERROR;
	}

	autoRunSetDefaults(modes);
	memcpy(state->modeByRun, modes, sizeof(modes));
	if (writeConfig == NULL ||
		!autoRunFormatConfig(modes, text, sizeof(text), &length) ||
		!writeConfig(context, text, length))
	{
		resetSeriesState(state);
		return AUTO_RUN_CONFIG_LOAD_REPAIR_FAILED;
	}
	state->configReady = true;
	resetSeriesState(state);
	return AUTO_RUN_CONFIG_LOAD_READY;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunModeName
// 処理概要     方式列挙値に対応するログ用文字列を取得する
// 引数         mode: 走行方式
// 戻り値       方式名文字列
/////////////////////////////////////////////////////////////////////
const char *autoRunModeName(AutoRunMode mode)
{
	switch (mode)
	{
	case AUTO_RUN_MODE_PRIMARY: return "PRIMARY";
	case AUTO_RUN_MODE_DISTANCE: return "DISTANCE";
	case AUTO_RUN_MODE_SLIP: return "SLIP";
	case AUTO_RUN_MODE_PATH: return "PATH";
	case AUTO_RUN_MODE_SHORTCUT: return "SHORTCUT";
	default: return "INVALID";
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunBeginSeries
// 処理概要     オートスタート5走分のログ参照状態を初期化する
// 引数         state: オートスタート状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void autoRunBeginSeries(AutoRunState *state)
{
	if (state == NULL)
	{
		return;
	}
	resetSeriesState(state);
	state->seriesActive = state->configReady;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunAbortSeries
// 処理概要     オートスタート系列のログ参照状態を破棄する
// 引数         state: オートスタート状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void autoRunAbortSeries(AutoRunState *state)
{
	if (state != NULL)
	{
		resetSeriesState(state);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunPrepareRun
// 処理概要     走行番号に応じた方式と一次・スリップ参照ログを明示的に選ぶ
// 引数         state: オートスタート状態, runNumber: 走行番号, plan: 実行計画格納先
// 戻り値       true: 実行可能 false: 設定または必要ログが不正
/////////////////////////////////////////////////////////////////////
bool autoRunPrepareRun(AutoRunState *state, uint8_t runNumber, AutoRunPlan *plan)
{
	AutoRunPlan prepared = {0U, AUTO_RUN_MODE_INVALID, 0, 0};

	if (state == NULL || plan == NULL || !state->configReady || runNumber < 1U || runNumber > 5U)
	{
		return false;
	}
	if (runNumber == 1U)
	{
		if (!state->seriesActive)
		{
			autoRunBeginSeries(state);
		}
		prepared.requestedMode = AUTO_RUN_MODE_PRIMARY;
	}
	else
	{
		if (!state->seriesActive || state->firstLogNumber <= 0 || state->previousLogNumber <= 0)
		{
			return false;
		}
		prepared.requestedMode = state->modeByRun[runNumber - 2U];
		prepared.primaryLogNumber = state->firstLogNumber;
		if (prepared.requestedMode == AUTO_RUN_MODE_SLIP)
		{
			if (state->previousMode != AUTO_RUN_MODE_DISTANCE)
			{
				return false;
			}
			prepared.slipSourceLogNumber = state->previousLogNumber;
		}
	}
	prepared.runNumber = runNumber;
	*plan = prepared;
	state->currentRunNumber = runNumber;
	state->currentMode = prepared.requestedMode;
	state->currentPrimaryLogNumber = prepared.primaryLogNumber;
	state->slipSourceLogNumber = prepared.slipSourceLogNumber;
	state->currentPlanValid = true;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunCompleteRun
// 処理概要     正常終了して保存された走行だけを次走の参照元に登録する
// 引数         state: オートスタート状態, plan: 実行計画, savedLogNumber: 保存ログ番号, normalEnd: 正常終了, logSaved: 保存成功
// 戻り値       true: 次走の準備可能 false: 次走を停止すべき
/////////////////////////////////////////////////////////////////////
bool autoRunCompleteRun(AutoRunState *state, const AutoRunPlan *plan, int16_t savedLogNumber, bool normalEnd, bool logSaved)
{
	if (state == NULL || plan == NULL || !state->currentPlanValid ||
		plan->runNumber != state->currentRunNumber || plan->requestedMode != state->currentMode ||
		!normalEnd || !logSaved || savedLogNumber <= 0)
	{
		return false;
	}
	if (plan->runNumber == 1U)
	{
		state->firstLogNumber = savedLogNumber;
		state->previousMode = AUTO_RUN_MODE_PRIMARY;
	}
	else
	{
		state->previousMode = plan->requestedMode;
	}
	state->previousLogNumber = savedLogNumber;
	state->currentPlanValid = false;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 autoRunPrimaryLogForHeader
// 処理概要     ログヘッダへ記録する一次走行ログ番号を取得する
// 引数         state: オートスタート状態, currentLogNumber: 1走目の保存番号
// 戻り値       参照する一次ログ番号。不正時は0
/////////////////////////////////////////////////////////////////////
int16_t autoRunPrimaryLogForHeader(const AutoRunState *state, int16_t currentLogNumber)
{
	if (state == NULL || !state->currentPlanValid || state->currentMode == AUTO_RUN_MODE_INVALID)
	{
		return 0;
	}
	if (state->currentRunNumber == 1U)
	{
		return currentLogNumber;
	}
	return state->currentPrimaryLogNumber;
}
