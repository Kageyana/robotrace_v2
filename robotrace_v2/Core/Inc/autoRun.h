#ifndef AUTORUN_H_
#define AUTORUN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUTO_RUN_CONFIG_SLOT_COUNT 4U
#define AUTO_RUN_CONFIG_TEXT_CAPACITY 128U

typedef enum
{
	AUTO_RUN_MODE_PRIMARY = 0,
	AUTO_RUN_MODE_DISTANCE,
	AUTO_RUN_MODE_SLIP,
	AUTO_RUN_MODE_PATH,
	AUTO_RUN_MODE_SHORTCUT,
	AUTO_RUN_MODE_INVALID
} AutoRunMode;

typedef struct
{
	AutoRunMode modeByRun[AUTO_RUN_CONFIG_SLOT_COUNT];
	bool configReady;
	bool seriesActive;
	bool currentPlanValid;
	uint8_t currentRunNumber;
	AutoRunMode previousMode;
	AutoRunMode currentMode;
	int16_t firstLogNumber;
	int16_t previousLogNumber;
	int16_t slipSourceLogNumber;
	int16_t currentPrimaryLogNumber;
} AutoRunState;

typedef struct
{
	uint8_t runNumber;
	AutoRunMode requestedMode;
	int16_t primaryLogNumber;
	int16_t slipSourceLogNumber;
} AutoRunPlan;

typedef enum
{
	// 設定ファイル読込コールバックの結果
	AUTO_RUN_CONFIG_READ_SUCCESS = 0,
	AUTO_RUN_CONFIG_READ_MISSING,
	AUTO_RUN_CONFIG_READ_INVALID,
	AUTO_RUN_CONFIG_READ_IO_ERROR
} AutoRunConfigReadResult;

typedef enum
{
	// 設定ロード処理の最終結果
	AUTO_RUN_CONFIG_LOAD_READY = 0,
	AUTO_RUN_CONFIG_LOAD_REPAIR_FAILED,
	AUTO_RUN_CONFIG_LOAD_IO_ERROR
} AutoRunConfigLoadResult;

typedef AutoRunConfigReadResult (*AutoRunConfigRead)(void *context, char *buffer, size_t capacity, size_t *length);
typedef bool (*AutoRunConfigWrite)(void *context, const char *text, size_t length);

extern AutoRunState autoRunState;

void autoRunSetDefaults(AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT]);
bool autoRunParseConfig(const char *text, AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT]);
bool autoRunFormatConfig(const AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT], char *buffer, size_t capacity, size_t *length);
AutoRunConfigLoadResult autoRunLoadConfig(AutoRunState *state, AutoRunConfigRead readConfig, AutoRunConfigWrite writeConfig, void *context);
const char *autoRunModeName(AutoRunMode mode);
void autoRunBeginSeries(AutoRunState *state);
void autoRunAbortSeries(AutoRunState *state);
bool autoRunPrepareRun(AutoRunState *state, uint8_t runNumber, AutoRunPlan *plan);
bool autoRunCompleteRun(AutoRunState *state, const AutoRunPlan *plan, int16_t savedLogNumber, bool normalEnd, bool logSaved);
int16_t autoRunPrimaryLogForHeader(const AutoRunState *state, int16_t currentLogNumber);

#endif // AUTORUN_H_
