//====================================//
// 繧､繝ｳ繧ｯ繝ｫ繝ｼ繝・
//====================================//
#include "courseAnalysis.h"
#include "control.h"
#include "fatfs.h"
#include "PIDcontrol.h"
#include "markerSensor.h"
#include "BMI088.h"
#include "SDcard.h"
#include "sd_diskio_spi.h"
#include "sd_functions.h"
#include "ff.h"
#include <stdint.h>
#include <string.h>

static bool sd_remount_for_analysis(void)
{
	return (sd_remount() == FR_OK);
}
//====================================//
// 繧ｰ繝ｭ繝ｼ繝舌Ν螟画焚縺ｮ螳｣
//====================================//
uint8_t optimalTrace = 0;
uint16_t optimalIndex;
int16_t numPPADarry; // path palanning analysis distance (PPAD)
int16_t numPPAMarry; // path palanning analysis marker (PPAM1)
int16_t indexSC;
int16_t pathedMarker = 0;
float boostSpeed;
int32_t DistanceOptimal = 0; // 2谺｡襍ｰ陦檎畑襍ｰ陦瑚ｷ晞屬螟画焚
int16_t analyzedNumber = 0;	 // 蜑榊屓隗｣譫舌＠縺溘Ο繧ｰ逡ｪ蜿ｷ
int32_t encTotalOptimal = 0; // 2谺｡襍ｰ陦檎畑縺ｮ霍晞屬螟画焚(霍晞屬陬懈ｭ｣繧偵☆繧・
int32_t encPID = 0;			 // 霍晞屬蛻ｶ蠕｡逕ｨ縺ｮ霍晞屬螟画焚
float xydegz = 0;
int32_t straightMeter;
bool straightState;
bool straightMarkerPending;
uint8_t straightMarkerPendingLog;

static uint8_t missedCorrections = 0;	// 騾｣邯夊｣懈ｭ｣螟ｱ謨怜屓謨ｰ
static bool failSafeActive = false;	// 繝輔ぉ繧､繝ｫ繧ｻ繝ｼ繝募虚菴應ｸｭ繝輔Λ繧ｰ
static int16_t lastCorrectedMarker = 0;	// 逶ｴ霑代〒陬懈ｭ｣縺励◆繝槭・繧ｫ繝ｼ繧､繝ｳ繝・ャ繧ｯ繧ｹ

static void logReadSlipIoError(int logNumber, UINT lineNo, FIL *fil, const char *tag);
static float calcDecelLeadMmByRoc(int16_t rocPrev, int16_t rocNow);
static void applyDecelLeadToPpad(int16_t count);
static void applyDecelLeadToArray(float *speed, int16_t count);

typedef struct
{
	int16_t courseMarker;
	int16_t encTotalOptimal;
	int16_t ROC;
	int16_t targetSpeed;
	int16_t optimalIndex;
	int16_t slipFlag;
	int16_t slipFlagLat;
} SecondLogColumnMap;

static void initSecondLogColumnMap(SecondLogColumnMap *map)
{
	map->courseMarker = -1;
	map->encTotalOptimal = -1;
	map->ROC = -1;
	map->targetSpeed = -1;
	map->optimalIndex = -1;
	map->slipFlag = -1;
	map->slipFlagLat = -1;
}

static bool csvFieldEquals(const char *start, const char *end, const char *name)
{
	while (start < end && (*start == ' ' || *start == '\t'))
	{
		start++;
	}
	while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
	{
		end--;
	}

	size_t nameLength = strlen(name);
	return (size_t)(end - start) == nameLength && strncmp(start, name, nameLength) == 0;
}

static bool secondLogColumnMapIsValid(const SecondLogColumnMap *map)
{
	return map->courseMarker >= 0 && map->encTotalOptimal >= 0 && map->ROC >= 0 &&
		map->targetSpeed >= 0 && map->optimalIndex >= 0 && map->slipFlag >= 0 &&
		map->slipFlagLat >= 0;
}

static int16_t secondLogMaxRequiredColumn(const SecondLogColumnMap *map)
{
	int16_t maxColumn = map->courseMarker;
	if (map->encTotalOptimal > maxColumn) maxColumn = map->encTotalOptimal;
	if (map->ROC > maxColumn) maxColumn = map->ROC;
	if (map->targetSpeed > maxColumn) maxColumn = map->targetSpeed;
	if (map->optimalIndex > maxColumn) maxColumn = map->optimalIndex;
	if (map->slipFlag > maxColumn) maxColumn = map->slipFlag;
	if (map->slipFlagLat > maxColumn) maxColumn = map->slipFlagLat;
	return maxColumn;
}

static bool parseSecondLogHeader(const char *line, SecondLogColumnMap *map)
{
	initSecondLogColumnMap(map);

	const char *fieldStart = line;
	const char *p = line;
	int16_t column = 0;
	while (*p != '\0' && *p != '\n' && *p != '\r')
	{
		if (*p == ',')
		{
			if (csvFieldEquals(fieldStart, p, "courseMarker")) map->courseMarker = column;
			else if (csvFieldEquals(fieldStart, p, "encTotalOptimal")) map->encTotalOptimal = column;
			else if (csvFieldEquals(fieldStart, p, "ROC")) map->ROC = column;
			else if (csvFieldEquals(fieldStart, p, "targetSpeed")) map->targetSpeed = column;
			else if (csvFieldEquals(fieldStart, p, "optimalIndex")) map->optimalIndex = column;
			else if (csvFieldEquals(fieldStart, p, "slipFlag")) map->slipFlag = column;
			else if (csvFieldEquals(fieldStart, p, "slipFlagLat")) map->slipFlagLat = column;
			column++;
			fieldStart = p + 1;
		}
		p++;
	}

	if (csvFieldEquals(fieldStart, p, "courseMarker")) map->courseMarker = column;
	else if (csvFieldEquals(fieldStart, p, "encTotalOptimal")) map->encTotalOptimal = column;
	else if (csvFieldEquals(fieldStart, p, "ROC")) map->ROC = column;
	else if (csvFieldEquals(fieldStart, p, "targetSpeed")) map->targetSpeed = column;
	else if (csvFieldEquals(fieldStart, p, "optimalIndex")) map->optimalIndex = column;
	else if (csvFieldEquals(fieldStart, p, "slipFlag")) map->slipFlag = column;
	else if (csvFieldEquals(fieldStart, p, "slipFlagLat")) map->slipFlagLat = column;

	return secondLogColumnMapIsValid(map);
}

AnalysisData PPAD[OPT_BUFF_SIZE];
EventPos markerPos[OPT_BUFF_SIZE];
Courseplot xycie;							   // xy蠎ｧ讓吝､(襍ｰ陦御ｸｭ險育ｮ励√Ο繧ｰ菫晏ｭ倡畑)
Courseplot shortCutxycie[OPT_SHORT_BUFF_SIZE]; // xy蠎ｧ讓吝､(逶ｮ讓吝､縲√Ο繧ｰ菫晏ｭ倡畑)

/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・calcROC
// 蜃ｦ逅・ｦりｦ・    譖ｲ邇・濠蠕・・險育ｮ・
// 蠑墓焚         velo: 繧ｨ繝ｳ繧ｳ繝ｼ繝繧ｫ繧ｦ繝ｳ繝・angvelo: 隗帝溷ｺｦ[rad/s]
// 謌ｻ繧雁､       譖ｲ邇・濠蠕Ъmm]
/////////////////////////////////////////////////////////////////////
float calcROC(int16_t velo, float angvelo, float dt)
{
	// 遘ｻ蜍戊ｷ晞屬 [pulse] 竊・[mm]
    float dl = calcDlMm(velo, dt);  // [mm]
    // 隗貞ｺｦ螟牙喧驥・[rad] = ﾏ閏deg/s] 竊・rad/s ﾃ・dt[s]
    float drad = angvelo * DEG2RAD * dt;

    // 邨ｶ蟇ｾ蛟､繧堤ｬｦ蜿ｷ縺ｧ蜿悶ｋ fabs() 繧医ｊ鬮倬・
    float absDrad = (drad < 0.0f) ? -drad : drad;
    float absDl   = (dl   < 0.0f) ? -dl   : dl;

    // 逶ｴ邱壼愛螳夲ｼ嘶dl/drad| > ROC_STRAIGHT_TH 竍・ROC_STRAIGHT_TH * |drad| < |dl|
    // 竊・髯､邂励○縺壹↓豈碑ｼ・〒縺阪ｋ
    if (absDrad < 1e-6f || ROC_STRAIGHT_TH * absDrad < absDl) {
        return ROC_STRAIGHT_MAX; // 逶ｴ邱壹→縺ｿ縺ｪ縺・
    }

    // 繧ｫ繝ｼ繝悶・蝣ｴ蜷医・縺ｿ髯､邂怜ｮ溯｡・
    float R = absDl / absDrad;
    // float absR = (R < 0.0f) ? -R : R;
    // return (absR > ROC_STRAIGHT_TH) ? 2000.0f : R;

	return R;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・saveLogNumber
// 蜃ｦ逅・ｦりｦ・    隗｣譫舌＠縺溘Ο繧ｰ繝輔ぃ繧､繝ｫ縺ｮ逡ｪ蜿ｷ繧偵ヵ繧｡繧､繝ｫ縺ｫ菫晏ｭ倥☆繧・
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void saveLogNumber(int16_t fileNumber)
{
	FRESULT fresult;
	FIL fil;
	char fileName[32] = PATH_SETTING;

	strcat(fileName, FILENAME_ANALYSIS_NUMBER);					 // 繝輔ぃ繧､繝ｫ蜷崎ｿｽ蜉
	strcat(fileName, ".txt");									 // 諡｡蠑ｵ蟄占ｿｽ蜉
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // create file
	if (fresult == FR_OK)
	{
		f_lseek(&fil, 0);
		f_truncate(&fil);
		f_printf(&fil, "%05d", fileNumber);
		f_close(&fil);
	}
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・getLogNumber
// 蜃ｦ逅・ｦりｦ・    隗｣譫舌＠縺溘Ο繧ｰ繝輔ぃ繧､繝ｫ縺ｮ逡ｪ蜿ｷ繧貞叙蠕励☆繧・
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void getLogNumber(void)
{
	FRESULT fresult;
	FIL fil;
	TCHAR log[20];
	char fileName[32] = PATH_SETTING;
	int parsedNumber = analyzedNumber;
	bool repair = false;

	strcat(fileName, FILENAME_ANALYSIS_NUMBER);					// 繝輔ぃ繧､繝ｫ蜷崎ｿｽ蜉
	strcat(fileName, ".txt");									// 諡｡蠑ｵ蟄占ｿｽ蜉
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // csv繝輔ぃ繧､繝ｫ繧帝幕縺・
	if (fresult == FR_OK)
	{
		// 隗｣譫先ｸ医∩縺ｮ繝ｭ繧ｰ逡ｪ蜿ｷ繧貞叙蠕・
		if (f_gets(log, (int)(sizeof(log) / sizeof(log[0])), &fil) != NULL &&
			sscanf(log, "%5d", &parsedNumber) == 1 &&
			parsedNumber >= 0 && parsedNumber <= INT16_MAX)
		{
			analyzedNumber = (int16_t)parsedNumber;
		}
		else
		{
			repair = true;
		}
		f_close(&fil);
	}
	else
	{
		repair = true;
	}

	if (repair)
	{
		saveLogNumber(analyzedNumber);
	}

	for (int16_t i = 0; i <= endFileIndex; i++)
	{
		// 隗｣譫先ｸ医∩縺ｮ繝ｭ繧ｰ逡ｪ蜿ｷ縺ｫ荳閾ｴ縺吶ｋ繧､繝ｳ繝・ャ繧ｯ繧ｹ繧剃ｿ晏ｭ・
		if (analyzedNumber == fileNumbers[i])
		{
			fileIndexLog = i;
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ sortInt16Ascending
// 蜃ｦ逅・ｦりｦ・    譛螟ｧ5隕∫ｴ縺ｮint16_t驟榊・繧呈諺蜈･繧ｽ繝ｼ繝医☆繧九％縺ｨ縺ｧqsort蜻ｼ縺ｳ蜃ｺ縺励ｒ蜑頑ｸ・
// 蠑墓焚         values: 繧ｽ繝ｼ繝亥ｯｾ雎｡縺ｮ驟榊・, length: 隕∫ｴ謨ｰ
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
static void sortInt16Ascending(int16_t *values, uint16_t length)
{
	if (length <= 1)
	{
		return; // 隕∫ｴ謨ｰ1莉･荳九・荳ｦ縺ｹ譖ｿ縺井ｸ崎ｦ・
	}

	for (uint16_t index = 1; index < length; index++)
	{
		int16_t key = values[index];
		uint16_t insertPos = index;
		while (insertPos > 0 && values[insertPos - 1] > key)
		{
			values[insertPos] = values[insertPos - 1];
			insertPos--;
		}
		values[insertPos] = key;
	}
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・calcDecelLeadMmByRoc
// 蜃ｦ逅・ｦりｦ・    譖ｲ邇・濠蠕・・螟牙喧驥上°繧牙・陦梧ｸ幃溯ｷ晞屬[mm]繧堤ｮ怜・縺吶ｋ
// 蠑墓焚         rocPrev:逶ｴ蜑阪・譖ｲ邇・濠蠕Ъmm], rocNow:迴ｾ蝨ｨ縺ｮ譖ｲ邇・濠蠕Ъmm]
// 謌ｻ繧雁､       蜈郁｡梧ｸ幃溯ｷ晞屬[mm]
/////////////////////////////////////////////////////////////////////
static float calcDecelLeadMmByRoc(int16_t rocPrev, int16_t rocNow)
{
	float baseLeadMm = tgtParam.decelLeadMm;
	int16_t absPrev = (int16_t)abs(rocPrev);
	int16_t absNow = (int16_t)abs(rocNow);

	if (baseLeadMm <= 0.0f)
	{
		return 0.0f;
	}
	if (absPrev <= absNow)
	{
		return 0.0f; // 譖ｲ邇・′邱ｩ縺上↑繧区婿蜷代・蜈郁｡梧ｸ幃溘＠縺ｪ縺・
	}
	if (absPrev <= 0)
	{
		absPrev = 1;
	}

	float changeRatio = (float)(absPrev - absNow) / (float)absPrev; // 螟牙喧邇・0.0縲・.0)
	if (changeRatio < 0.0f)
	{
		changeRatio = 0.0f;
	}
	if (changeRatio > 1.0f)
	{
		changeRatio = 1.0f;
	}

	return baseLeadMm * changeRatio;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・applyDecelLeadToPpad
// 蜃ｦ逅・ｦりｦ・    PPAD騾溷ｺｦ驟榊・縺ｮ貂幃溷玄髢薙↓蜈郁｡梧ｸ幃溘ｒ驕ｩ逕ｨ縺吶ｋ
// 蠑墓焚         count:PPAD驟榊・縺ｮ譛牙柑隕∫ｴ謨ｰ
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
static void applyDecelLeadToPpad(int16_t count)
{
	if (count <= 1)
	{
		return;
	}

	for (int16_t i = 1; i < count; i++)
	{
		float prevSpeed = PPAD[i - 1].boostSpeed;
		float nowSpeed = PPAD[i].boostSpeed;
		if (nowSpeed >= prevSpeed)
		{
			continue; // do not shift acceleration
		}

		float leadMm = calcDecelLeadMmByRoc(PPAD[i - 1].ROC, PPAD[i].ROC);
		if (leadMm <= 0.0f)
		{
			continue;
		}

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // mm繧帝・蛻励せ繝・ャ繝玲焚縺ｸ謠帷ｮ・
		if (leadStep <= 0)
		{
			continue;
		}

		int16_t start = i - leadStep;
		if (start < 0)
		{
			start = 0;
		}
		for (int16_t j = start; j < i; j++)
		{
			// 貂幃溷ｾ碁溷ｺｦ縺後☆縺ｧ縺ｫ豎ｺ縺ｾ繧倶ｽ咲ｽｮ繧呈焔蜑榊・縺ｸ諡｡蠑ｵ
			if (PPAD[j].boostSpeed > nowSpeed)
			{
				PPAD[j].boostSpeed = nowSpeed;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・applyDecelLeadToArray
// 蜃ｦ逅・ｦりｦ・    莉ｻ諢上・騾溷ｺｦ驟榊・縺ｮ貂幃溷玄髢薙↓蜈郁｡梧ｸ幃溘ｒ驕ｩ逕ｨ縺吶ｋ
// 蠑墓焚         speed:騾溷ｺｦ驟榊・, count:譛牙柑隕∫ｴ謨ｰ
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
static void applyDecelLeadToArray(float *speed, int16_t count)
{
	if (speed == NULL || count <= 1)
	{
		return;
	}

	for (int16_t i = 1; i < count; i++)
	{
		float prevSpeed = speed[i - 1];
		float nowSpeed = speed[i];
		if (nowSpeed >= prevSpeed)
		{
			continue; // do not shift acceleration
		}

		float leadMm = calcDecelLeadMmByRoc(PPAD[i - 1].ROC, PPAD[i].ROC);
		if (leadMm <= 0.0f)
		{
			continue;
		}

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // mm繧帝・蛻励せ繝・ャ繝玲焚縺ｸ謠帷ｮ・
		if (leadStep <= 0)
		{
			continue;
		}

		int16_t start = i - leadStep;
		if (start < 0)
		{
			start = 0;
		}
		for (int16_t j = start; j < i; j++)
		{
			// 貂幃溷ｾ碁溷ｺｦ縺後☆縺ｧ縺ｫ豎ｺ縺ｾ繧倶ｽ咲ｽｮ繧呈焔蜑榊・縺ｸ諡｡蠑ｵ
			if (speed[j] > nowSpeed)
			{
				speed[j] = nowSpeed;
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・readLogDistance
// 蜃ｦ逅・ｦりｦ・    霍晞屬蝓ｺ貅・谺｡襍ｰ陦後・隗｣譫・
// 蠑墓焚         繝ｭ繧ｰ逡ｪ蜿ｷ(繝輔ぃ繧､繝ｫ蜷・
// 謌ｻ繧雁､       譛驕ｩ騾溷ｺｦ驟榊・縺ｮ譛螟ｧ隕∫ｴ謨ｰ
//              -1: 隗｣譫千畑驟榊・縺ｮ繧ｵ繧､繧ｺ繧定ｶ・℃
//              -4: 繝ｭ繧ｰ繝輔ぃ繧､繝ｫ縺ｮ繧ｪ繝ｼ繝励Φ螟ｱ謨・
/////////////////////////////////////////////////////////////////////
int16_t readLogDistance(int logNumber)
{
	// 繝輔ぃ繧､繝ｫ隱ｭ縺ｿ霎ｼ縺ｿ
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool fileOpened = false; // f_close
	bool retried = false;
	bool errorDetected = false; // 隗｣譫宣比ｸｭ縺ｮ繧ｨ繝ｩ繝ｼ逋ｺ逕溘ｒ讀懃衍縺吶ｋ繝輔Λ繧ｰ
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9;
	}
	// 隗｣譫蝉ｸｭ縺ｯ繝ｭ繧ｰ譖ｸ縺崎ｾｼ縺ｿ繧呈椛蛻ｶ縺吶ｋ
	sd_set_analysis_active(true); // SD/FatFs菴ｿ逕ｨ荳ｭ
	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 謨ｰ蛟､繧呈枚蟄怜・縺ｫ螟画鋤
	strcat(fileName, ".csv");										   // 諡｡蠑ｵ蟄舌ｒ霑ｽ蜉
	retry_open:
	// 隗｣譫仙燕縺ｫ蜀阪・繧ｦ繝ｳ繝医＠縺ｦFAT縺ｮ謨ｴ蜷医ｒ蜿悶ｊ逶ｴ縺・
	if (!sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup_read;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csv繝輔ぃ繧､繝ｫ繧帝幕縺・

	if (fresult == FR_OK)
	{
		fileOpened = true; // 豁｣蟶ｸ縺ｫ髢九￠縺溷ｴ蜷医・縺ｿ繧ｯ繝ｭ繝ｼ繧ｺ蜃ｦ逅・ｒ譛牙柑蛹・
		// 繝ｭ繧ｰ繝・・繧ｿ縺ｮ蜿門ｾ・
		TCHAR log[512];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t time, marker, velo, distance, roc, i = 0;
		float angVelo;
		int32_t numD = 0, numM = 0, cntCurR = 0, numStraight = 0;
		static int16_t ROCbuff[600] = {0};
		int16_t sortROC[CALCDISTANCE / 10];	// sortROC縺ｮ譛螟ｧ隕∫ｴ謨ｰ縺ｯCALCDISTANCE/10(=5)縲ょ虚逧・｢ｺ菫昴→繝・ヰ繝・げprintf繧呈賜髯､縺吶ｋ縺溘ａ閾ｪ蜍暮・蛻励ｒ蛻ｩ逕ｨ
		int32_t straightMeter = 0;
		bool straightState = false;

		// 蜑榊・逅・
		// 讒矩菴馴・蛻励・蛻晄悄蛹・
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		TCHAR *header = f_gets(log, log_len, &fil_Read); // 1陦檎岼縺ｯ繝倥ャ繝縺ｪ縺ｮ縺ｧ隱ｭ縺ｿ鬟帙・縺・
		if (!header && f_error(&fil_Read))
		{
			ret = -5;
			errorDetected = true;
			logReadSlipIoError(logNumber, 0, &fil_Read, "io_fail");
		}

		UINT lineNo = 0;
		// 繝ｭ繧ｰ繝・・繧ｿ蜿門ｾ鈴幕蟋・
		while (!errorDetected)
		{
			TCHAR *s = f_gets(log, log_len, &fil_Read);
			if (!s)
			{
				if (f_error(&fil_Read))
				{
					ret = -5;
					errorDetected = true;
					logReadSlipIoError(logNumber, lineNo, &fil_Read, "io_fail");
				}
				break;
			}
			lineNo++;

			sscanf(log, "%d,%d,%f,%d,%d,%d,", &time, &velo, &angVelo, &marker, &distance, &roc);
			// 隗｣譫仙・逅・
			// marker==3: 莠､蟾ｮ邱壹・繝ｼ繧ｫ繝ｼ
			// marker==2: 蟾ｦ繝槭・繧ｫ繝ｼ縲ら峩邱夊ｵｰ陦御ｸｭ縺ｮ縺ｿ繧ｫ繝ｼ繝悶・繝ｼ繧ｫ繝ｼ縺ｨ縺励※謇ｱ縺・
			if (marker == 3 || (marker == 2 && straightState))
			{
				// 繧ｫ繝ｼ繝悶・繝ｼ繧ｫ繝ｼ繧帝夐℃縺励◆縺ｨ縺阪↓繝槭・繧ｫ繝ｼ菴咲ｽｮ繧定ｨ倬鹸
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;

				if (marker == 2 && straightState)
				{
				// 逶ｴ邱壼ｾ後・蟾ｦ繝槭・繧ｫ繝ｼ讀懷・縺ｧ繝輔Λ繧ｰ縺ｨ霍晞屬繧偵Μ繧ｻ繝・ヨ
					straightState = false;
					straightMeter = 0;
				}

				numM++; // 繝槭・繧ｫ繝ｼ隗｣譫舌う繝ｳ繝・ャ繧ｯ繧ｹ譖ｴ譁ｰ
			}

			// 荳螳夊ｷ晞屬縺斐→縺ｫ蜃ｦ逅・
			if (i > 0 && i % (CALCDISTANCE / 10) == 0) // i==0縺ｧ縺ｯ蜃ｦ逅・＠縺ｪ縺・
			{
				int32_t copyCount = cntCurR;	    // 莉雁屓繧ｽ繝ｼ繝医☆繧玖ｦ∫ｴ謨ｰ繧帝驕ｿ
				if (copyCount > (CALCDISTANCE / 10))
				{
					copyCount = (CALCDISTANCE / 10); // 逅・ｫ紋ｸ雁芦驕斐＠縺ｪ縺・′縲∝ｮ牙・縺ｮ縺溘ａ荳企剞繧帝←逕ｨ
				}
				for (int32_t sortIndex = 0; sortIndex < copyCount; sortIndex++)
				{
					sortROC[sortIndex] = ROCbuff[sortIndex]; // 蠢・ｦ√↑隕∫ｴ縺ｮ縺ｿ繧呈焔蜍輔さ繝斐・縺励※荳ｭ螟ｮ蛟､邂怜・逕ｨ縺ｫ騾驕ｿ
				}
				sortInt16Ascending(sortROC, (uint16_t)copyCount); // 蟆城・蛻励・蜊倡ｴ斐た繝ｼ繝医〒蜊∝・縺ｪ縺溘ａqsort蜻ｼ縺ｳ蜃ｺ縺励ｒ蜑頑ｸ・

				// 譖ｲ邇・濠蠕・ｒ險倬鹸縺吶ｋ
				if (copyCount % 2 == 0)
				{
					// 荳ｭ螟ｮ蛟､繧定ｨ倬鹸(驟榊・隕∫ｴ謨ｰ縺悟・謨ｰ縺ｮ縺ｨ縺・ 荳ｭ螟ｮ2縺､縺ｮ蟷ｳ蝮・､
					PPAD[numD].ROC = (sortROC[copyCount / 2] + sortROC[copyCount / 2 - 1]) / 2;
				}
				else
				{
					// 荳ｭ螟ｮ蛟､繧定ｨ倬鹸(驟榊・隕∫ｴ謨ｰ縺悟･・焚縺ｮ縺ｨ縺・
					PPAD[numD].ROC = sortROC[copyCount / 2];
				}

				PPAD[numD].boostSpeed = asignVelocity(PPAD[numD].ROC); // 譖ｲ邇・濠蠕・＃縺ｨ縺ｮ騾溷ｺｦ繧定ｨ育ｮ励☆繧・

				// 蜑榊屓縺ｮ譖ｲ邇・濠蠕・→豈碑ｼ・numD縺・莉･荳翫・蝣ｴ蜷医・縺ｿ)
				if (numD >= 1 && PPAD[numD].ROC == PPAD[numD - 1].ROC)
				{
					numStraight++;
				}
				else
				{
					numStraight = 0;
				}

				cntCurR = 0; // 譖ｲ邇・濠蠕・畑驟榊・縺ｮ繧ｫ繧ｦ繝ｳ繝医け繝ｪ繧｢
				numD++;		 // 霍晞屬隗｣譫舌う繝ｳ繝・ャ繧ｯ繧ｹ譖ｴ譁ｰ
				if (numD >= OPT_BUFF_SIZE)
				{
					ret = -1; // 隗｣譫千畑驟榊・縺ｮ繧ｵ繧､繧ｺ雜・℃繧呈､懷・縺励◆繧峨お繝ｩ繝ｼ謇ｱ縺・→縺吶ｋ
					errorDetected = true; // 繧ｨ繝ｩ繝ｼ繝輔Λ繧ｰ繧堤ｫ九※縺ｦ蜈ｱ騾壹け繝ｪ繝ｼ繝ｳ繧｢繝・・縺ｸ驕ｷ遘ｻ
					break; // 蜊ｳ譎Ｓeturn縺帙★繝ｫ繝ｼ繝励ｒ謚懊￠繧・
				}
			}
			// 譖ｲ邇・濠蠕・・險育ｮ・
			ROCbuff[cntCurR] = roc;

			if (abs(ROCbuff[cntCurR]) >= 700)
			{
				straightMeter += CALCDISTANCE_SHORTCUT;
			}
			else
			{
				straightMeter = 0;
			}

			// 逶ｴ邱壼玄髢薙′100mm莉･荳顔ｶ壹＞縺溘ｉ逶ｴ邱夊ｵｰ陦御ｸｭ縺ｨ蛻､螳壹＠縲・
			// 谺｡縺ｫ讀懷・縺吶ｋ蟾ｦ繝槭・繧ｫ繝ｼ繧偵き繝ｼ繝夜幕蟋九→縺吶ｋ縺溘ａ縺ｮ繝輔Λ繧ｰ繧堤ｫ九※繧・
			if (straightMeter >= 100)
			{
				straightState = true;
			}

			cntCurR++; // 譖ｲ邇・濠蠕・畑驟榊・縺ｮ繧ｫ繧ｦ繝ｳ繝・
			i++;
		}

		if (!errorDetected)
		{
			// 繧､繝ｳ繝・ャ繧ｯ繧ｹ縺・螟壹￥縺ｪ繧九・縺ｧ隱ｿ謨ｴ
			if (numM > 0)
			{
				numM--;        // 0莉ｶ譎ゅ・繝槭・繧ｫ繝ｼ謨ｰ繧定ｲ縺ｫ縺励↑縺・
			}
			int32_t numDCount = 0;
			if (numD > 0)
			{
				numD--;        // 0莉ｶ譎ゅ・霍晞屬隕∫ｴ謨ｰ繧定ｲ縺ｫ縺励↑縺・
				numDCount = numD + 1;        // 隕∫ｴ謨ｰ縺ｫ謌ｻ縺励※蜉貂幃溯ｪｿ謨ｴ縺ｧ菴ｿ逕ｨ
			}
			else
			{
				numDCount = numD;        // 隕∫ｴ謨ｰ0縺ｮ蝣ｴ蜷医・縺昴・縺ｾ縺ｾ蛻ｩ逕ｨ
			}
			numD = numDCount;
			applyDecelLeadToPpad((int16_t)numD); // 譖ｲ邇・､牙喧縺ｫ蠢懊§縺ｦ縲∵ｸ幃溷芦驕比ｽ咲ｽｮ繧呈焔蜑阪∈蟇・○繧・

			// 逶ｮ讓咎溷ｺｦ驟榊・縺ｮ謨ｴ蠖｢ 蜉貂幃溘′髢薙↓蜷医≧繧医≧縺ｫ霍晞屬繧定ｪｿ謨ｴ縺吶ｋ
			float acceleration, elapsedTime, dv, dl;

			// 譛蛻昴・隕∫ｴ縺ｯ隱ｿ謨ｴ縺励↑縺・
			dl = (float)CALCDISTANCE / 1000;

			// numD繧剃ｻｶ謨ｰ縺ｨ縺励※謇ｱ縺・◆繧√∽ｻ･荳九・繝ｫ繝ｼ繝励〒繧ょ｢・阜螟悶い繧ｯ繧ｻ繧ｹ縺ｯ逋ｺ逕溘＠縺ｪ縺・

			// 蜉騾溘う繝ｳ繝・ャ繧ｯ繧ｹ1縺九ｉ譛ｫ蟆ｾ縺ｾ縺ｧ蟷ｳ貊大喧
			for (int32_t idx = 1; idx < numD; idx++)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx - 1].boostSpeed);	// 蛹ｺ髢馴溷ｺｦ蟾ｮ
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 騾溷ｺｦ蟾ｮ縺梧･ｵ蟆上↑繧芽｣懈ｭ｣荳崎ｦ・
				}
				elapsedTime = fabs(dl / dv);		// 蛹ｺ髢捺凾髢・
				acceleration = dv / elapsedTime;	// 螳滓ｸｬ蜉騾溷ｺｦ
				if (acceleration > MACHINEACCELE)
				{
					PPAD[idx].boostSpeed = PPAD[idx - 1].boostSpeed + (MACHINEACCELE * dl);
				}
			}

			// 貂幃溘う繝ｳ繝・ャ繧ｯ繧ｹ譛ｫ蟆ｾ縺九ｉ蜈磯ｭ縺ｾ縺ｧ蟷ｳ貊大喧
			for (int32_t idx = numD - 2; idx >= 0; idx--)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx + 1].boostSpeed);	// 蛹ｺ髢馴溷ｺｦ蟾ｮ
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 騾溷ｺｦ蟾ｮ縺梧･ｵ蟆上↑繧芽｣懈ｭ｣荳崎ｦ・
				}
				elapsedTime = fabs(dl / dv);
				acceleration = dv / elapsedTime;
				if (acceleration > MACHINEDECREACE)
				{
					PPAD[idx].boostSpeed = PPAD[idx + 1].boostSpeed + (MACHINEDECREACE * dl);
				}
			}

#ifdef WRITE_BOOSTSPEED_LOG
			// 蟷ｳ貊大喧蠕後・逶ｮ讓咎溷ｺｦ驟榊・繧担D繧ｫ繝ｼ繝峨∈險倬鹸縺吶ｋ
			FIL fil_Boost;
			FRESULT fresult_Boost;
			char boostFileName[32];
			snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
			fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
			if (fresult_Boost == FR_OK)
			{
				// CSV繝倥ャ繝繧呈嶌縺崎ｾｼ縺ｿ縲∝ｹｳ貊大喧貂医∩縺ｮboostSpeed繧帝・分縺ｫ菫晏ｭ倥☆繧・
				UINT bytesWritten;
				f_printf(&fil_Boost, "index,boost_speed\n");
				for (int32_t idx = 0; idx < numD; idx++)
				{
					char boostLine[48];

					// f_printf縺ｯ%f髱槫ｯｾ蠢懊・縺溘ａ縲・陦悟・繧呈枚蟄怜・縺ｫ謨ｴ蠖｢縺励※縺九ｉ譖ｸ縺崎ｾｼ繧
					snprintf(boostLine, sizeof(boostLine), "%ld,%.3f\n", (long)idx, PPAD[idx].boostSpeed);
					f_write(&fil_Boost, boostLine, strlen(boostLine), &bytesWritten);
				}
				f_close(&fil_Boost);
			}
#endif

			numPPAMarry = numM;
			numPPADarry = numD;
			ret = numD;
		}
		else
		{
			// 繧ｨ繝ｩ繝ｼ逋ｺ逕滓凾縺ｯ謨ｴ蠖｢蜃ｦ逅・ｒ陦後ｏ縺夊ｧ｣譫千ｵ先棡繧堤ｴ譽・
		}
	}
	else
	{
		ret = -4;
		errorDetected = true; // 繝輔ぃ繧､繝ｫ繧ｪ繝ｼ繝励Φ螟ｱ謨玲凾繧ゅお繝ｩ繝ｼ迥ｶ諷九→縺励※謇ｱ縺・
	}

	if (ret == -5 && !retried)
	{
		// I/O繧ｨ繝ｩ繝ｼ譎ゅ・荳蠎ｦ縺縺大・繝槭え繝ｳ繝茨ｼ・・繧ｪ繝ｼ繝励Φ繧定ｩｦ縺・
		if (fileOpened)
		{
			f_close(&fil_Read);
			fileOpened = false;
		}
		retried = true;
		ret = 0;
		errorDetected = false;
		goto retry_open;
	}

cleanup_read:
	if (fileOpened)
	{
		f_close(&fil_Read); // 繧ｪ繝ｼ繝励Φ謌仙粥譎ゅ・縺ｿ繧ｯ繝ｭ繝ｼ繧ｺ繧貞ｮ滓命
	}
	if (lock_acquired)
	{
		// 隗｣譫千ｵゆｺ・ｼ域嶌縺崎ｾｼ縺ｿ謚大宛隗｣髯､・・
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	// printf("Analysis distance end\n");

	if (ret >= 0)
	{
		// 豁｣蟶ｸ邨ゆｺ・凾縺ｮ縺ｿ隗｣譫先ｸ医∩諠・ｱ繧呈峩譁ｰ
		saveLogNumber(logNumber);
		analyzedNumber = logNumber;

		// 2谺｡襍ｰ陦後ヵ繝ｩ繧ｰ 霍晞屬蝓ｺ貅・谺｡襍ｰ陦・
		optimalTrace = BOOST_DISTANCE;
	}
	else
	{
		// 繧ｨ繝ｩ繝ｼ逋ｺ逕滓凾縺ｯ迥ｶ諷区峩譁ｰ繧定｡後ｏ縺壼他縺ｳ蜃ｺ縺怜・縺ｫ霑泌唆
	}

	return ret;
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ parseSecondLogLine
// 蜃ｦ逅・ｦりｦ・ 2谺｡襍ｰ陦後Ο繧ｰ縺ｮ蠢・ｦ∝・縺ｮ縺ｿ繧呈歓蜃ｺ縺吶ｋ
// 蠑墓焚	 line: 1陦梧枚蟄怜・, 蜷・・蜉帛・繝昴う繝ｳ繧ｿ
// 謌ｻ繧雁､	 隗｣譫先・蜉溘↑繧液rue
/////////////////////////////////////////////////////////////////////
static bool parseSecondLogLine(const char *line, const SecondLogColumnMap *map,
		uint8_t *courseMarker, int32_t *encTotal, int16_t *roc,
		float *targetSpeedLog, int16_t *optimalIdx, uint8_t *slipLong, uint8_t *slipLat)
{
	// 霑ｽ蜉: 繝倥ャ繝/遨ｺ陦悟愛螳壹・縺溘ａ蜈磯ｭ縺ｮ譛牙柑譁・ｭ励ｒ遒ｺ隱阪☆繧・
	const char *p = line;
	while (*p == ' ' || *p == '\t')
	{
		p++;
	}
	if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '+'))
	{
		return false;	// 謨ｰ蛟､縺ｧ蟋九∪繧峨↑縺・｡後・繧ｹ繧ｭ繝・・
	}

	// 霑ｽ蜉: 繧ｫ繝ｳ繝槭ｒ謨ｰ縺医↑縺後ｉ蠢・ｦ∝・縺縺代ｒ謚ｽ蜃ｺ縺吶ｋ
	int col = 0;
	const int16_t maxColumn = secondLogMaxRequiredColumn(map);
	const char *field = p;
	bool gotOptimal = false;
	while (1)
	{
		// 霑ｽ蜉: 蛹ｺ蛻・ｊ譁・ｭ・繧ｫ繝ｳ繝・謾ｹ陦・邨らｫｯ)縺ｧ繝輔ぅ繝ｼ繝ｫ繝峨ｒ遒ｺ螳壹☆繧・
		if (*p == ',' || *p == '\n' || *p == '\r' || *p == '\0')
		{
			char *endptr = NULL;
			if (col == map->courseMarker)
				*courseMarker = (uint8_t)strtol(field, &endptr, 10);
			else if (col == map->encTotalOptimal)
				*encTotal = (int32_t)strtol(field, &endptr, 10);
			else if (col == map->ROC)
				*roc = (int16_t)strtol(field, &endptr, 10);
			else if (col == map->targetSpeed)
				*targetSpeedLog = strtof(field, &endptr);
			else if (col == map->optimalIndex)
			{
				*optimalIdx = (int16_t)strtol(field, &endptr, 10);
				gotOptimal = true;
			}
			else if (col == map->slipFlag)
				*slipLong = (uint8_t)strtol(field, &endptr, 10);
			else if (col == map->slipFlagLat)
				*slipLat = (uint8_t)strtol(field, &endptr, 10);
			if (*p == ',')
			{
				col++;
				if (col > maxColumn)
				{
					break;	// 霑ｽ蜉: 蠢・ｦ∝・繧定ｶ・∴縺溘ｉ譌ｩ譛溽ｵゆｺ・
				}
				p++;
				field = p;
				continue;
			}
			break;
		}
		p++;
	}

	return gotOptimal;
}

static void logReadSlipIoError(int logNumber, UINT lineNo, FIL *fil, const char *tag)
{
	FIL fil_Boost;
	FRESULT fresult_Boost;
	char boostFileName[32];
	DWORD pos = f_tell(fil);
	DWORD size = f_size(fil);
	int eof = f_eof(fil);
	int err = f_error(fil);

	snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
	fresult_Boost = f_open(&fil_Boost, boostFileName, FA_OPEN_ALWAYS | FA_WRITE);
	if (fresult_Boost == FR_OK)
	{
		// 繝輔ぃ繧､繝ｫ邨らｫｯ縺ｸ遘ｻ蜍・繝輔ぃ繧､繝ｫ霑ｽ險倥・貅門ｙ)
		f_lseek(&fil_Boost, f_size(&fil_Boost));
		f_printf(&fil_Boost,
			"readLogDistanceSlip %s: line=%lu pos=%lu size=%lu eof=%d err=%d sd_sector=%lu sd_count=%u sd_rb=%d sd_rm=%d\n",
			(tag != NULL) ? tag : "io",
			(unsigned long)lineNo, (unsigned long)pos, (unsigned long)size, eof, err,
			(unsigned long)g_sd_last_read_sector, (unsigned int)g_sd_last_read_count,
			g_sd_last_read_blocks_status, g_sd_last_read_multi_status);
		f_close(&fil_Boost);
	}
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・readLogDistanceSlip
// 蜃ｦ逅・ｦりｦ・    2谺｡襍ｰ陦後Ο繧ｰ縺九ｉ繧ｹ繝ｪ繝・・繧定・・縺励◆3谺｡襍ｰ陦檎畑騾溷ｺｦ險育判繧剃ｽ懈・縺吶ｋ
// 蠑墓焚         繝ｭ繧ｰ逡ｪ蜿ｷ(繝輔ぃ繧､繝ｫ蜷・
// 謌ｻ繧雁､       譛驕ｩ騾溷ｺｦ驟榊・縺ｮ譛螟ｧ隕∫ｴ謨ｰ
/////////////////////////////////////////////////////////////////////
int16_t readLogDistanceSlip(int logNumber)
{
	int16_t baseLogNumber = analyzedNumber;
	if (baseLogNumber <= 0)
	{
		baseLogNumber = logNumber; // 霑ｽ蜉: 1襍ｰ逶ｮ繝ｭ繧ｰ縺檎┌縺代ｌ縺ｰ逶ｴ蜑阪Ο繧ｰ繧剃ｽｿ逕ｨ
	}

	// 霑ｽ蜉: 1襍ｰ逶ｮ繝ｭ繧ｰ繧池eadLogDistance逶ｸ蠖薙〒隗｣譫舌＠縺ｦ騾溷ｺｦ險育判縺ｨ繝槭・繧ｫ繝ｼ驟榊・繧剃ｽ懈・
	int16_t baseRet = readLogDistance(baseLogNumber);
	if (baseRet < 0)
	{
		return baseRet;
	}
	int16_t baseCount = numPPADarry;
	if (baseCount <= 0)
	{
		return -2; // 隗｣譫仙ｯｾ雎｡縺檎┌縺・
	}

	FIL fil_Read;
	FRESULT fresult;
	char fileName[16];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);
	bool fileOpened = false;
	bool retried = false;

	if (!lock_acquired)
	{
		return -9; // SD/FatFs菴ｿ逕ｨ荳ｭ
	}
	// 隗｣譫蝉ｸｭ縺ｯ繝ｭ繧ｰ譖ｸ縺崎ｾｼ縺ｿ繧呈椛蛻ｶ縺吶ｋ
	sd_set_analysis_active(true);

	// 霑ｽ蜉: 隗｣譫千畑繝舌ャ繝輔ぃ繝ｻ驟榊・縺ｯ髱咏噪鬆伜沺縺ｧ遒ｺ菫昴＠縺ｦ繧ｹ繧ｿ繝・け繧堤ｯ邏・
	static uint16_t sampleCnt[OPT_BUFF_SIZE];
	static float v2Max[OPT_BUFF_SIZE];
	static float rocAbsSum[OPT_BUFF_SIZE];
	static uint16_t rocCnt[OPT_BUFF_SIZE];
	static uint16_t slipLongCnt[OPT_BUFF_SIZE];
	static uint16_t slipLatCnt[OPT_BUFF_SIZE];
	static float risk[OPT_BUFF_SIZE];
	static float riskExpanded[OPT_BUFF_SIZE];
	static float v3[OPT_BUFF_SIZE];

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 謨ｰ蛟､繧呈枚蟄怜・縺ｫ螟画鋤
	strcat(fileName, ".csv");										   // 諡｡蠑ｵ蟄舌ｒ霑ｽ蜉
	retry_open_slip:
	// 隗｣譫仙燕縺ｫ蜀阪・繧ｦ繝ｳ繝医＠縺ｦFAT縺ｮ謨ｴ蜷医ｒ蜿悶ｊ逶ｴ縺・
	if (!sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csv繝輔ぃ繧､繝ｫ繧帝幕縺・
	if (fresult != FR_OK)
	{
		ret = -4; // 繝ｭ繧ｰ繝輔ぃ繧､繝ｫ縺ｮ繧ｪ繝ｼ繝励Φ螟ｱ謨・
		goto cleanup;
	}
	fileOpened = true;

	memset(sampleCnt, 0, sizeof(sampleCnt));
	memset(v2Max, 0, sizeof(v2Max));
	memset(rocAbsSum, 0, sizeof(rocAbsSum));
	memset(rocCnt, 0, sizeof(rocCnt));
	memset(slipLongCnt, 0, sizeof(slipLongCnt));
	memset(slipLatCnt, 0, sizeof(slipLatCnt));
	memset(risk, 0, sizeof(risk));
	memset(riskExpanded, 0, sizeof(riskExpanded));
	memset(v3, 0, sizeof(v3));
	for (int16_t i = 0; i < baseCount && i < OPT_BUFF_SIZE; i++)
	{
		v2Max[i] = PPAD[i].boostSpeed;
	}

	static TCHAR log[CA_SECOND_LOG_LINE_BUFSIZE];
	const int log_len = (int)(sizeof(log) / sizeof(log[0]));
	int16_t maxOptimalIndex = -1;
	SecondLogColumnMap secondLogColumns;

	// 1陦檎岼縺ｯ繝倥ャ繝縺ｪ縺ｮ縺ｧ隱ｭ縺ｿ鬟帙・縺・
	TCHAR *header = f_gets(log, log_len, &fil_Read);
	if (!header)
	{
		int eof = f_eof(&fil_Read);
		int err = f_error(&fil_Read);
		if (!eof && err != 0)
		{
			ret = -5; // f_gets I/O error
			logReadSlipIoError(logNumber, 0, &fil_Read, "io_fail");
		}
		else
		{
			ret = -2; // 隗｣譫仙ｯｾ雎｡縺檎┌縺・
		}
		goto cleanup;
	}
	if (!parseSecondLogHeader((const char *)header, &secondLogColumns))
	{
		ret = -2; // 2次ログに必要な列がない
		goto cleanup;
	}

	UINT lineNo = 0;
	bool fgets_null = false;

	while (1) {
		TCHAR* s = f_gets(log, log_len, &fil_Read);
		if (!s)
		{
			fgets_null = true;
			break;
		}
		lineNo++;

		if (f_error(&fil_Read))
		{
			logReadSlipIoError(logNumber, lineNo, &fil_Read, "io_err");
		}

		uint8_t courseMarker = 0;
		int32_t encTotal = 0;
		int16_t roc = 0;
		float targetSpeedLog = 0.0f;
		int16_t optimalIdx = 0;
		uint8_t slipLong = 0;
		uint8_t slipLat = 0;

		if (!parseSecondLogLine((const char *)log, &secondLogColumns,
				&courseMarker, &encTotal, &roc,
				&targetSpeedLog, &optimalIdx, &slipLong, &slipLat))
		{
			continue;	// 霑ｽ蜉: 繝倥ャ繝/遨ｺ陦後・隗｣譫舌＠縺ｪ縺・
		}
		if (optimalIdx < 0 || optimalIdx >= baseCount)
		{
			ret = -1;	// 霑ｽ蜉: 隗｣譫千畑驟榊・縺ｮ荳企剞雜・℃
			break;
		}

		// 霑ｽ蜉: 髮・ｨ・繧ｵ繝ｳ繝励Ν謨ｰ/譛螟ｧ騾溷ｺｦ/ROC蟷ｳ蝮・繧ｹ繝ｪ繝・・蝗樊焚)
		(void)courseMarker;
		(void)targetSpeedLog;
		(void)encTotal;

		sampleCnt[optimalIdx]++;
		rocAbsSum[optimalIdx] += fabsf((float)roc);
		rocCnt[optimalIdx]++;
		if (slipLong > 0)
		{
			slipLongCnt[optimalIdx]++;
		}
		if (slipLat > 0)
		{
			slipLatCnt[optimalIdx]++;
		}

		if (optimalIdx > maxOptimalIndex)
		{
			maxOptimalIndex = optimalIdx;
		}

		// 霑ｽ蜉: 2谺｡繝ｭ繧ｰ縺九ｉ繝槭・繧ｫ繝ｼ菴咲ｽｮ繧貞・讒狗ｯ峨☆繧・
	}
	if (ret == 0 && fgets_null)
	{
		int eof = f_eof(&fil_Read);
		int err = f_error(&fil_Read);
		if (!eof && err != 0)
		{
			ret = -5; // f_gets I/O error
			logReadSlipIoError(logNumber, lineNo, &fil_Read, "io_fail");
		}
	}

	if (ret == -5 && !retried)
	{
		// I/O繧ｨ繝ｩ繝ｼ譎ゅ・荳蠎ｦ縺縺大・繝槭え繝ｳ繝茨ｼ・・繧ｪ繝ｼ繝励Φ繧定ｩｦ縺・
		if (fileOpened)
		{
			f_close(&fil_Read);
			fileOpened = false;
		}
		retried = true;
		ret = 0;
		goto retry_open_slip;
	}

cleanup:
	if (fileOpened)
	{
		f_close(&fil_Read);
	}
	if (lock_acquired)
	{
		// 隗｣譫千ｵゆｺ・ｼ域嶌縺崎ｾｼ縺ｿ謚大宛隗｣髯､・・
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	if (ret < 0)
	{
		return ret;
	}
	if (maxOptimalIndex < 0)
	{
		return -2;	// 霑ｽ蜉: 隗｣譫仙ｯｾ雎｡縺檎┌縺・
	}

	// 霑ｽ蜉: 谺逡ｪoptimalIndex縺ｯ蜑榊､縺ｧ蝓九ａ繧・蜑肴婿蝓九ａ)
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		if (sampleCnt[i] == 0 && i > 0)
		{
			rocAbsSum[i] = rocAbsSum[i - 1];	// 霑ｽ蜉: ROC蟷ｳ蝮・畑縺ｮ蜑肴婿蝓九ａ
			rocCnt[i] = rocCnt[i - 1];			// 霑ｽ蜉: ROC蟷ｳ蝮・畑縺ｮ蜑肴婿蝓九ａ
		}
	}

	// 霑ｽ蜉: risk(0..1)繧剃ｽ懊ｋ
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		if (sampleCnt[i] == 0)
		{
			risk[i] = 0.0f;
			continue;
		}
		uint16_t longCnt = (slipLongCnt[i] >= CA_SLIP_CNT_MIN) ? slipLongCnt[i] : 0;
		uint16_t latCnt = (slipLatCnt[i] >= CA_SLIP_CNT_MIN) ? slipLatCnt[i] : 0;
		float fracLong = (float)longCnt / (float)sampleCnt[i];
		float fracLat = (float)latCnt / (float)sampleCnt[i];
		float riskLong = fracLong / CA_SLIP_FRAC_FULL;
		float riskLat = fracLat / CA_SLIP_FRAC_FULL;
		if (riskLong > 1.0f)
		{
			riskLong = 1.0f;
		}
		if (riskLat > 1.0f)
		{
			riskLat = 1.0f;
		}
		risk[i] = (riskLong > riskLat) ? riskLong : riskLat;
	}

	// 霑ｽ蜉: 霑大ｍ縺ｸ繝ｪ繧ｹ繧ｯ諡｡蠑ｵ
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		float expanded = risk[i];
		if (i - 1 >= 0)
		{
			float cand = risk[i - 1] * CA_SLIP_EXPAND_1;
			if (cand > expanded)
			{
				expanded = cand;
			}
		}
		if (i - 2 >= 0)
		{
			float cand = risk[i - 2] * CA_SLIP_EXPAND_2;
			if (cand > expanded)
			{
				expanded = cand;
			}
		}
		if (i + 1 <= maxOptimalIndex)
		{
			float cand = risk[i + 1] * CA_SLIP_EXPAND_1;
			if (cand > expanded)
			{
				expanded = cand;
			}
		}
		if (i + 2 <= maxOptimalIndex)
		{
			float cand = risk[i + 2] * CA_SLIP_EXPAND_2;
			if (cand > expanded)
			{
				expanded = cand;
			}
		}
		riskExpanded[i] = expanded;
	}

	// 霑ｽ蜉: v3繧呈峩譁ｰ(貂幃・蠅鈴・
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		float v = v2Max[i];
		if (riskExpanded[i] > 0.0f)
		{
			float scale = 1.0f - (CA_SLIP_DOWN_RISK * riskExpanded[i]);
			if (slipLongCnt[i] >= CA_SLIP_CNT_MIN)
			{
				scale -= CA_SLIP_DOWN_LONG_EXTRA;
			}
			if (slipLatCnt[i] >= CA_SLIP_CNT_MIN)
			{
				scale -= CA_SLIP_DOWN_LAT_EXTRA;
			}
			if (scale < CA_SLIP_MIN_SCALE)
			{
				scale = CA_SLIP_MIN_SCALE;
			}
			v3[i] = v * scale;
		}
		else
		{
			float avgRoc = (rocCnt[i] > 0) ? (rocAbsSum[i] / (float)rocCnt[i]) : ROC_STRAIGHT_TH;
			float up = (avgRoc >= ROC_STRAIGHT_TH) ? CA_SLIP_UP_STRAIGHT : CA_SLIP_UP_CURVE;
			v3[i] = v * (1.0f + up);
		}
	}

	// 霑ｽ蜉: 2谺｡縺ｮ騾溷ｺｦ螟牙喧驥丈ｻ･蜀・↓蜿弱ａ繧・蜑榊ｾ後ヱ繧ｹ)
	applyDecelLeadToArray(v3, (int16_t)(maxOptimalIndex + 1)); // 譖ｲ邇・､牙喧縺ｫ蠢懊§縺ｦ縲∵ｸ幃溷芦驕比ｽ咲ｽｮ繧呈焔蜑阪∈蟇・○繧・
	for (int16_t i = 0; i < maxOptimalIndex; i++)
	{
		float dvUp = v2Max[i + 1] - v2Max[i];
		if (dvUp < 0.0f)
		{
			dvUp = 0.0f;
		}
		float limit = v3[i] + dvUp;
		if (v3[i + 1] > limit)
		{
			v3[i + 1] = limit;
		}
	}
	for (int32_t i = maxOptimalIndex - 1; i >= 0; i--)
	{
		float dvDown = v2Max[i] - v2Max[i + 1];
		if (dvDown < 0.0f)
		{
			dvDown = 0.0f;
		}
		float limit = v3[i + 1] + dvDown;
		if (v3[i] > limit)
		{
			v3[i] = limit;
		}
	}

	// 霑ｽ蜉: PPAD縺ｸ蜿肴丐
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		PPAD[i].boostSpeed = v3[i];
	}

	ret = numPPADarry;

#ifdef WRITE_BOOSTSPEED_LOG
	// 蟷ｳ貊大喧蠕後・逶ｮ讓咎溷ｺｦ驟榊・繧担D繧ｫ繝ｼ繝峨∈險倬鹸縺吶ｋ
	FIL fil_Boost;
	FRESULT fresult_Boost;
	char boostFileName[32];
	snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
	fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
	if (fresult_Boost == FR_OK)
	{
		// CSV繝倥ャ繝繧呈嶌縺崎ｾｼ縺ｿ縲∝ｹｳ貊大喧貂医∩縺ｮboostSpeed繧帝・分縺ｫ菫晏ｭ倥☆繧・
		UINT bytesWritten;
	f_printf(&fil_Boost, "index,boost_speed\n");
	for (int32_t idx = 0; idx < maxOptimalIndex; idx++)
	{
		char boostLine[48];

			// f_printf縺ｯ%f髱槫ｯｾ蠢懊・縺溘ａ縲・陦悟・繧呈枚蟄怜・縺ｫ謨ｴ蠖｢縺励※縺九ｉ譖ｸ縺崎ｾｼ繧
			snprintf(boostLine, sizeof(boostLine), "%ld,%.3f\n", (long)idx, PPAD[idx].boostSpeed);
			f_write(&fil_Boost, boostLine, strlen(boostLine), &bytesWritten);
		}
		f_close(&fil_Boost);
	}
#endif

	// 霑ｽ蜉: 隗｣譫先ｸ医∩諠・ｱ繧呈峩譁ｰ
	optimalTrace = BOOST_DISTANCE;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・asignVelocity
// 蜃ｦ逅・ｦりｦ・    譖ｲ邇・濠蠕・＃縺ｨ縺ｮ譛驕ｩ騾溷ｺｦ繧貞牡繧雁ｽ薙※繧・
// 蠑墓焚
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
float asignVelocity(int16_t ROC)
{
	int16_t absROC;
	float ret;

	absROC = abs(ROC);
	if (absROC > 1500)
		ret = tgtParam.bstStraight;
	if (absROC <= 1500)
		ret = tgtParam.bst1500;
	if (absROC <= 1300)
		ret = tgtParam.bst1300;
	if (absROC <= 1000)
		ret = tgtParam.bst1000;
	if (absROC <= 800)
		ret = tgtParam.bst800;
	if (absROC <= 700)
		ret = tgtParam.bst700;
	if (absROC <= 600)
		ret = tgtParam.bst600;
	if (absROC <= 500)
		ret = tgtParam.bst500;
	if (absROC <= 400)
		ret = tgtParam.bst400;
	if (absROC <= 300)
		ret = tgtParam.bst300;
	if (absROC <= 200)
		ret = tgtParam.bst200;
	if (absROC <= 100)
		ret = tgtParam.bst100;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・cmpfloat
// 蜃ｦ逅・ｦりｦ・    float蝙九・豈碑ｼ・
// 蠑墓焚
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
int cmpfloat(const void *n1, const void *n2)
{
	if (*(float *)n1 > *(float *)n2)
		return 1;
	else if (*(float *)n1 < *(float *)n2)
		return -1;
	else
		return 0;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・readLogDistance
// 蜃ｦ逅・ｦりｦ・    霍晞屬蝓ｺ貅・谺｡襍ｰ陦後・隗｣譫・
// 蠑墓焚         繝ｭ繧ｰ逡ｪ蜿ｷ(繝輔ぃ繧､繝ｫ蜷・
// 謌ｻ繧雁､       譛驕ｩ騾溷ｺｦ驟榊・縺ｮ譛螟ｧ隕∫ｴ謨ｰ
/////////////////////////////////////////////////////////////////////
int16_t readLogTest(int logNumber)
{
	// 繝輔ぃ繧､繝ｫ隱ｭ縺ｿ霎ｼ縺ｿ
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9; // SD/FatFs菴ｿ逕ｨ荳ｭ
	}

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 謨ｰ蛟､繧呈枚蟄怜・縺ｫ螟画鋤
	strcat(fileName, ".csv");										   // 諡｡蠑ｵ蟄舌ｒ霑ｽ蜉
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csv繝輔ぃ繧､繝ｫ繧帝幕縺・

	if (fresult == FR_OK)
	{
		TCHAR log[512];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t time, marker, velo, distance;
		float angVelo;
		int32_t startEnc = 0, numD = 0, numM = 0, beforeMarker = 0;
		bool analysis = false;

		// 蜑榊・逅・
		// 讒矩菴馴・蛻励・蛻晄悄蛹・
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		// 繝ｭ繧ｰ繝・・繧ｿ蜿門ｾ鈴幕蟋・
		while (f_gets(log, log_len, &fil_Read))
		{
			sscanf(log, "%d,%d,%f,%d,%d", &time, &velo, &angVelo, &marker, &distance);

			// 隗｣譫仙・逅・
			if (marker == 1 && beforeMarker == 0)
			{
				// 繧ｴ繝ｼ繝ｫ繝槭・繧ｫ繝ｼ繧帝夐℃縺励◆縺ｨ縺阪↓繝輔Λ繧ｰ蜿崎ｻ｢
				analysis = !analysis;
				startEnc = distance;
			}
			else if (marker == 0 && beforeMarker == 2)
			{
				// 繧ｫ繝ｼ繝悶・繝ｼ繧ｫ繝ｼ繧帝夐℃縺励◆縺ｨ縺阪↓繝槭・繧ｫ繝ｼ菴咲ｽｮ繧定ｨ倬鹸
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;
				numM++; // 繝槭・繧ｫ繝ｼ隗｣譫舌う繝ｳ繝・ャ繧ｯ繧ｹ譖ｴ譁ｰ
			}
			if (!analysis && startEnc > 0)
				break;
			numD++;
		}
		ret = numD;
	}
	else
	{
		ret = -1;
	}
	f_close(&fil_Read);
	if (lock_acquired)
	{
		sd_fatfs_unlock();
	}

	// 隗｣譫先ｸ医∩縺ｮ繝ｭ繧ｰ逡ｪ蜿ｷ繧剃ｿ晏ｭ・
	// saveLogNumber(logNumber);
	analyzedNumber = logNumber;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・calcXYcies (cie=Coordinate)
// 蜃ｦ逅・ｦりｦ・    繝ｭ繧ｰ縺九ｉ襍ｰ陦瑚ｻ瑚ｷ｡縺ｮXY蠎ｧ讓吶ｒ險育ｮ励☆繧・
// 蠑墓焚         繝ｭ繧ｰ逡ｪ蜿ｷ(繝輔ぃ繧､繝ｫ蜷・
// 謌ｻ繧雁､       隗｣譫舌＠縺滄・蛻励・隕∫ｴ謨ｰ
/////////////////////////////////////////////////////////////////////
int16_t calcXYcies(int logNumber)
{
	FIL fil_Read, fil_Plot;
	FRESULT fresult1, fresult2;
	char fileName[10];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9; // SD/FatFs菴ｿ逕ｨ荳ｭ
	}

	// 繝輔ぃ繧､繝ｫ隱ｭ縺ｿ霎ｼ縺ｿ
	snprintf(fileName, sizeof(fileName), "%d", logNumber);				// 謨ｰ蛟､繧呈枚蟄怜・縺ｫ螟画鋤
	strcat(fileName, ".csv");											// 諡｡蠑ｵ蟄舌ｒ霑ｽ蜉
	fresult1 = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // 繝ｭ繧ｰ繝輔ぃ繧､繝ｫ繧帝幕縺・
	if (fresult1 != FR_OK)
	{
		// 繝ｭ繧ｰ繝輔ぃ繧､繝ｫ縺ｮ繧ｪ繝ｼ繝励Φ縺ｫ螟ｱ謨励＠縺溷ｴ蜷医・繧ｨ繝ｩ繝ｼ繧定ｿ斐☆
		ret = -5;       // 繝ｭ繧ｰ繝輔ぃ繧､繝ｫ繧ｪ繝ｼ繝励Φ繧ｨ繝ｩ繝ｼ
		if (lock_acquired)
		{
			sd_fatfs_unlock();
		}
		return ret;
	}
	fresult2 = f_open(&fil_Plot, "./plot/plot.csv", FA_CREATE_ALWAYS | FA_WRITE); // csv繝輔ぃ繧､繝ｫ繧帝幕縺・
	if (fresult2 != FR_OK)
	{
		// 繝励Ο繝・ヨ繝輔ぃ繧､繝ｫ縺ｮ繧ｪ繝ｼ繝励Φ縺ｫ螟ｱ謨励＠縺溷ｴ蜷医・繧ｨ繝ｩ繝ｼ繧定ｿ斐☆
		f_close(&fil_Read);
		ret = -6;       // 繝励Ο繝・ヨ繝輔ぃ繧､繝ｫ繧ｪ繝ｼ繝励Φ繧ｨ繝ｩ繝ｼ
		if (lock_acquired)
		{
			sd_fatfs_unlock();
		}
		return ret;
	}

	// 繝励Ο繝・ヨ繝輔ぃ繧､繝ｫ縺碁幕縺代◆縺ｮ縺ｧ隗｣譫舌ｒ髢句ｧ・
	// 繝ｭ繧ｰ繝・・繧ｿ縺ｮ蜿門ｾ・
	TCHAR log[512];
	const int log_len = (int)(sizeof(log) / sizeof(log[0]));
	uint8_t plotStr[128];
	int32_t time, marker, velo, distance;
	float angVelo;
	int32_t beforeTime = 0, startEnc = 0, distEnc = 0;
	float degz = 0, degzR, velocity = 0, dt;
	float x = 0, y = 0, xm = 0, ym = 0, degzm = 0;
	float xValues[SHORTCUTWINDOW], yValues[SHORTCUTWINDOW], degzValues[SHORTCUTWINDOW];
	int16_t i = 0, j = 0;

	// 驟榊・縺ｮ蛻晄悄蛹・
	memset(&xValues, 0, sizeof(float) * SHORTCUTWINDOW);
	memset(&yValues, 0, sizeof(float) * SHORTCUTWINDOW);
	memset(&degzValues, 0, sizeof(float) * SHORTCUTWINDOW);
	indexSC = 0;

	// 繧ｷ繝ｧ繝ｼ繝医き繝・ヨ霆瑚ｷ｡蛻晄悄蛟､縺ｮ險ｭ螳・
	shortCutxycie[indexSC].x = 0;
	shortCutxycie[indexSC].y = 0;
	shortCutxycie[indexSC].w = 0;
	indexSC++;

	// plot繝輔ぃ繧､繝ｫ縺ｮ繝倥ャ繝譖ｸ縺崎ｾｼ縺ｿ
	f_printf(&fil_Plot, "xm,ym,degzm\n");
	
	f_gets(log, log_len, &fil_Read); // 1陦檎岼縺ｯ繝倥ャ繝縺ｪ縺ｮ縺ｧ隱ｭ縺ｿ鬟帙・縺・

	// 繝ｭ繧ｰ繝・・繧ｿ蜿門ｾ鈴幕蟋・
	while (f_gets(log, log_len, &fil_Read) != NULL)
	{
		sscanf(log, "%d,%d,%f,%d,%d", &time, &velo, &angVelo, &marker, &distance);

		dt = (float)(time - beforeTime) / 1000;		// 譎る俣[s]

		degz = degz + (angVelo * dt);			   	// 隗貞ｺｦ
		degzR = degz * DEG2RAD;					   	// [rad]縺ｫ螟画鋤
		velocity = (float)velo / PULSE_MILLIMETER;	// 騾溷ｺｦ
		distEnc += velo * (time - beforeTime);		// 霍晞屬險域ｸｬ

		// 蠎ｧ讓呵ｨ育ｮ・
		x = x + (velocity * sin(degzR));
		y = y + (velocity * cos(degzR));

		// 繝ｪ繝ｳ繧ｰ繝舌ャ繝輔ぃ縺ｫ蠎ｧ讓吶ｒ菫晏ｭ・
		xValues[i & (SHORTCUTWINDOW - 1)] = x;
		yValues[i & (SHORTCUTWINDOW - 1)] = y;
		degzValues[i & (SHORTCUTWINDOW - 1)] = degz;

		// 繝ｪ繝ｳ繧ｰ繝舌ャ繝輔ぃ縺ｮ邱丞柱險育ｮ怜燕縺ｫ蛻晄悄蛹・
		xm = ym = degzm = 0.0f; // 蜷・捉蝗槭〒豁｣縺励＞蟷ｳ蝮・､繧貞ｾ励ｋ縺溘ａ繝ｪ繧ｻ繝・ヨ
		// 繝ｪ繝ｳ繧ｰ繝舌ャ繝輔ぃ縺ｮ邱丞柱繧定ｨ育ｮ・
		for (j = 0; j < SHORTCUTWINDOW; j++)
		{
			xm += xValues[j];
			ym += yValues[j];
			degzm += degzValues[j];
		}

		// 遘ｻ蜍募ｹｳ蝮・ｒ險育ｮ・繧ｷ繝ｧ繝ｼ繝医き繝・ヨ蠎ｧ讓・
		xm /= SHORTCUTWINDOW;
		ym /= SHORTCUTWINDOW;
		degzm /= SHORTCUTWINDOW;
		if (distEnc - startEnc >= encMM(CALCDISTANCE_SHORTCUT))
		{
			// 繝舌ャ繝輔ぃ荳企剞縺ｫ驕斐＠縺ｦ縺・↑縺・°遒ｺ隱・
			if (indexSC < OPT_SHORT_BUFF_SIZE)
			{
				shortCutxycie[indexSC].x = xm;
				shortCutxycie[indexSC].y = ym;
				startEnc = distEnc; // 霍晞屬險域ｸｬ髢句ｧ倶ｽ咲ｽｮ繧呈峩譁ｰ
				indexSC++; // 繝舌ャ繝輔ぃ縺ｮ谺｡縺ｮ菴咲ｽｮ縺ｸ
			}
			else
			{
				// 荳企剞雜・℃: 繧ｨ繝ｩ繝ｼ逡ｪ蜿ｷ-7繧定ｨｭ螳壹＠繝ｫ繝ｼ繝励ｒ邨ゆｺ・
				ret = -7;
				break;
			}
		}

		i++;
		beforeTime = time;
	}

	// 繧ｷ繝ｧ繝ｼ繝医き繝・ヨ蠎ｧ讓吶°繧謁aw霆ｸ隗貞ｺｦ繧定ｨ育ｮ・
	float xe = 0, ye = 0;
	float theta = 0, thetaBefore = 90, thetae;

	degz = 0;
	// plot繝輔ぃ繧､繝ｫ縺ｫ蛻晄悄蛟､險倬鹸
	f_printf(&fil_Plot, "%d,%d,%d\n", (int32_t)(shortCutxycie[0].x * 10000), (int32_t)(shortCutxycie[0].y * 10000), (int32_t)(shortCutxycie[0].w * 10000));

	for (i = 1; i < indexSC; i++)
	{
		xe = shortCutxycie[i].x - shortCutxycie[i - 1].x; // x蠎ｧ讓吶・遘ｻ蜍暮㍼
		ye = shortCutxycie[i].y - shortCutxycie[i - 1].y; // y蠎ｧ讓吶・遘ｻ蜍暮㍼

		theta = atan2(ye, xe) * RAD2DEG; // [deg]縺ｫ螟画鋤

		// 2逶ｴ邱壹・縺ｪ縺呵ｧ偵ｒ險育ｮ・
		thetae = thetaBefore - theta;
		if (thetae > 180)
		{
			thetae -= 360;
		}
		else if (thetae < -180)
		{
			thetae += 360;
		}
		degz += thetae;

		shortCutxycie[i].w = degz; // yaw霆ｸ隗貞ｺｦ
		// plot繝輔ぃ繧､繝ｫ縺ｫ譖ｸ縺崎ｾｼ縺ｿ
		int snlen = snprintf((char *)plotStr, sizeof(plotStr), "%f,%f,%f\n", shortCutxycie[i].x, shortCutxycie[i].y, shortCutxycie[i].w); // 謌ｻ繧雁､縺ｧ譖ｸ縺崎ｾｼ縺ｿ髟ｷ繧堤｢ｺ隱・
		if (snlen < 0 || snlen >= sizeof(plotStr))
		{
			// snprintf縺悟､ｱ謨励＠縺溷ｴ蜷医ｄ繝舌ャ繝輔ぃ縺御ｸ崎ｶｳ縺励◆蝣ｴ蜷医・繧ｨ繝ｩ繝ｼ逡ｪ蜿ｷ-8繧定ｨｭ螳壹＠縺ｦ蜃ｦ逅・ｒ荳ｭ譁ｭ縺吶ｋ
			ret = -8;
			break;
		}
		f_puts((TCHAR *)plotStr, &fil_Plot);
		
		thetaBefore = theta; // 蜑榊屓縺ｮyaw霆ｸ隗貞ｺｦ繧呈峩譁ｰ
	}

	if (ret >= 0)
	{
		// 繝ｫ繝ｼ繝怜・縺ｧ繧ｨ繝ｩ繝ｼ縺後↑縺代ｌ縺ｰ隗｣譫舌＠縺溯ｦ∫ｴ謨ｰ繧定ｿ斐☆
		ret = indexSC;
	}

	// 繝輔ぃ繧､繝ｫ繧ｯ繝ｭ繝ｼ繧ｺ
	f_close(&fil_Read);
	f_close(&fil_Plot);
	if (lock_acquired)
	{
		sd_fatfs_unlock();
	}

	// 繧ｨ繝ｩ繝ｼ譎ゅ↓縺ｯ繝ｭ繧ｰ逡ｪ蜿ｷ菫晏ｭ倥ｄ繝輔Λ繧ｰ險ｭ螳壹ｒ繧ｹ繧ｭ繝・・縺吶ｋ
	if (ret >= 0)
	{
		// 隗｣譫先ｸ医∩縺ｮ繝ｭ繧ｰ逡ｪ蜿ｷ繧剃ｿ晏ｭ・
		saveLogNumber(logNumber);
		analyzedNumber = logNumber;

		// 2谺｡襍ｰ陦後ヵ繝ｩ繧ｰ 霍晞屬蝓ｺ貅・谺｡襍ｰ陦・
		optimalTrace = BOOST_SHORTCUT;
	}

	return ret;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・calcXYcie (cie=Coordinate)
// 蜃ｦ逅・ｦりｦ・    襍ｰ陦御ｸｭ縺ｫxy蠎ｧ讓吶ｒ險育ｮ励＠繧ｰ繝ｭ繝ｼ繝舌Ν螟画焚縺ｫ菫晏ｭ倥☆繧・
// 蠑墓焚         encpulse:繧ｨ繝ｳ繧ｳ繝ｼ繝繝代Ν繧ｹ angVelo:隗帝溷ｺｦ[deg/s]
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void calcXYcie(int16_t encpulse, float angVelo, float dt)
{
	static float velocity, degzR;

	xydegz = xydegz + (angVelo * dt);		// 隗貞ｺｦ
	degzR = xydegz * (M_PI / 180.0F);		// [rad]縺ｫ螟画鋤
	velocity = (float)encpulse / PULSE_MILLIMETER * 1000; // 騾溷ｺｦ

	xycie.x = xycie.x + (velocity * sin(degzR) * dt);
	xycie.y = xycie.y + (velocity * cos(degzR) * dt);
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・clearXYcie (cie=Coordinate)
// 蜃ｦ逅・ｦりｦ・    繧ｰ繝ｭ繝ｼ繝舌Ν螟画焚xycie縺ｮ蛻晄悄蛹・
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void clearXYcie(void)
{
	xycie.x = 0;
	xycie.y = 0;
	xydegz = 0;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・setShortCutTarget
// 蜃ｦ逅・ｦりｦ・    繧ｰ繝ｭ繝ｼ繝舌Ν螟画焚xycie縺ｮ蛻晄悄蛹・
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void setShortCutTarget(void)
{
	float xe, ye, dist;
	setTargetAngle(shortCutxycie[optimalIndex].w);

	xe = shortCutxycie[optimalIndex].x - xycie.x;
	ye = shortCutxycie[optimalIndex].y - xycie.y;

	dist = sqrt(pow(xe, 2) + pow(ye, 2));

        setTargetDist(dist);
}

/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ clampMarkerIndex
// 蜃ｦ逅・ｦりｦ・ 繝槭・繧ｫ繝ｼ繧､繝ｳ繝・ャ繧ｯ繧ｹ繧貞ｮ牙・縺ｪ遽・峇縺ｫ蜿弱ａ繧・
// 蠑墓焚	 idx: 蛟呵｣懊う繝ｳ繝・ャ繧ｯ繧ｹ
// 謌ｻ繧雁､	 遽・峇蜀・↓繧ｯ繝ｩ繝ｳ繝励＠縺溘う繝ｳ繝・ャ繧ｯ繧ｹ
/////////////////////////////////////////////////////////////////////
static int16_t clampMarkerIndex(int16_t idx)
{
	if (numPPAMarry <= 0)
	{
		return 0;
	}
	if (idx < 0)
	{
		return 0;
	}
	if (idx >= numPPAMarry)
	{
		return numPPAMarry - 1;
	}
	return idx;
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ isStraightBeforeMarker
// 蜃ｦ逅・ｦりｦ・ 逶ｴ蜑榊玄髢薙・逶ｴ邱夂紫繧貞愛螳壹☆繧・
// 蠑墓焚	 encNow: 迴ｾ蝨ｨ繧ｨ繝ｳ繧ｳ繝ｼ繝蛟､, window_mm: 隧穂ｾ｡遯甜mm], ratio_threshold: 逶ｴ邱夂紫髢ｾ蛟､
// 謌ｻ繧雁､	 髢ｾ蛟､莉･荳翫↑繧液rue
/////////////////////////////////////////////////////////////////////
static bool isStraightBeforeMarker(int32_t encNow, int16_t window_mm, float ratio_threshold)
{
	(void)encNow;
	int32_t straightDistance = straightMeter;	// 逶ｴ霑代〒逶ｴ邱壹→蛻､螳壹〒縺阪◆霍晞屬[mm]
	int32_t ratioScaled = (int32_t)(ratio_threshold * 1000.0f); // 髢ｾ蛟､繧貞崋螳壼ｰ乗焚轤ｹ(ﾃ・000)縺ｸ螟画鋤
	int32_t lhs = straightDistance * 1000;	// 隧穂ｾ｡遯薙↓蟇ｾ縺吶ｋ螳滄圀縺ｮ逶ｴ邱夂紫・亥・蟄仙・・・
	int32_t rhs = (int32_t)window_mm * ratioScaled;	// 髢ｾ蛟､ ﾃ・遯灘ｹ・ｼ亥・豈榊・繧貞酔蛟咲紫縺ｧ謠帷ｮ暦ｼ・
	return lhs >= rhs;	// 逶ｴ邱夂紫縺碁明蛟､莉･荳翫°縺ｩ縺・°蛻､螳・
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ calcDynamicThresholdPulse
// 蜃ｦ逅・ｦりｦ・ 陬懈ｭ｣險ｱ螳ｹ蛟､繧帝溷ｺｦ繝ｻ隗帝溷ｺｦ縺九ｉ蜍慕噪縺ｫ邂怜・縺吶ｋ
// 蠑墓焚	 縺ｪ縺・
// 謌ｻ繧雁､	 險ｱ螳ｹ霍晞屬[繝代Ν繧ｹ]
/////////////////////////////////////////////////////////////////////
static int32_t calcDynamicThresholdPulse(void)
{
	float speed_mm = encPulse(targetSpeed) * 1000;	// 騾溷ｺｦ[pulse]繧知m/s縺ｸ螟画鋤
	float mm = 100.0f + (CORR_DYN_COEFF_SPEED * speed_mm) + (CORR_DYN_COEFF_ANG * fabsf(imuVal.gyro.z));	// 蝓ｺ譛ｬ蛟､100mm縺ｫ騾溷ｺｦ繝ｻ隗帝溷ｺｦ縺ｮ陬懈ｭ｣繧貞刈邂・
	if (mm < (float)CORR_THRESH_MIN_MM)
	{
		mm = (float)CORR_THRESH_MIN_MM;	// 荳矩剞繧剃ｸ句屓繧峨↑縺・ｈ縺・け繝ｩ繝ｳ繝・
	}
	if (mm > (float)CORR_THRESH_MAX_MM)
	{
		mm = (float)CORR_THRESH_MAX_MM;	// 荳企剞繧定ｶ・∴縺溷ｴ蜷医・荳企剞縺ｧ蝗ｺ螳・
	}
	int16_t mmInt = (int16_t)(mm + 0.5f);	// 蝗帶昏莠泌・縺励※謨ｴ謨ｰmm縺ｸ
	return encMM(mmInt);	// mm竊偵ヱ繝ｫ繧ｹ縺ｸ謠帷ｮ励＠縺ｦ霑泌唆
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ findNearestMarkerIndex
// 蜃ｦ逅・ｦりｦ・ 霑大ｍ縺ｮ繝槭・繧ｫ繝ｼ縺九ｉ譛繧りｿ代＞繧､繝ｳ繝・ャ繧ｯ繧ｹ繧呈爾邏｢縺吶ｋ
// 蠑墓焚	 encNow: 迴ｾ蝨ｨ繧ｨ繝ｳ繧ｳ繝ｼ繝蛟､
// 謌ｻ繧雁､	 譛蟇・ｊ繝槭・繧ｫ繝ｼ縺ｮ繧､繝ｳ繝・ャ繧ｯ繧ｹ
/////////////////////////////////////////////////////////////////////
static int16_t findNearestMarkerIndex(int32_t encNow, bool isCross)
{
	if (numPPAMarry <= 0)
	{
		return 0;	// 繝槭・繧ｫ繝ｼ諠・ｱ縺檎┌縺・ｴ蜷医・蜈磯ｭ繧定ｿ斐☆
	}
	int16_t hint = clampMarkerIndex(pathedMarker);	// 謗ｨ螳夊ｵｰ陦御ｽ咲ｽｮ縺九ｉ縺ｮ繝偵Φ繝・
	int16_t center = clampMarkerIndex(lastCorrectedMarker);	// 逶ｴ霑代〒陬懈ｭ｣縺励◆繝槭・繧ｫ繝ｼ繧剃ｸｭ蠢・↓縺吶ｋ
	int16_t searchBack = isCross ? MARKER_SEARCH_CROSS_BACK : MARKER_SEARCH_BACK;
	int16_t searchForward = isCross ? MARKER_SEARCH_CROSS_FORWARD : MARKER_SEARCH_FORWARD;
	int16_t lower = center - searchBack;	// 蠕梧婿謗｢邏｢髢句ｧ倶ｽ咲ｽｮ
	int16_t upper = center + searchForward;	// 蜑肴婿謗｢邏｢邨ゆｺ・ｽ咲ｽｮ
	if (hint < lower)
	{
		lower = hint;	// 繝偵Φ繝医′繧医ｊ謇句燕縺ｪ繧牙ｾ梧婿遽・峇繧呈僑蠑ｵ
	}
	if (hint > upper)
	{
		upper = hint;	// 繝偵Φ繝医′蜈医↑繧牙燕譁ｹ遽・峇繧呈僑蠑ｵ
	}
	lower = clampMarkerIndex(lower);
	upper = clampMarkerIndex(upper);
	if (upper < lower)
	{
		int16_t tmp = upper;
		upper = lower;
		lower = tmp;	// 荳贋ｸ九′騾・ｻ｢縺励◆蝣ｴ蜷医・蜈･繧梧崛縺・
	}
	int16_t bestIdx = lower;	// 證ｫ螳壼呵｣懊ｒ荳矩剞縺ｫ險ｭ螳・
	int32_t bestDiff = encNow - markerPos[lower].distance;
	bestDiff = (bestDiff < 0) ? -bestDiff : bestDiff;
	for (int16_t idx = lower + 1; idx <= upper; idx++)
	{
		int32_t diff = encNow - markerPos[idx].distance;
		diff = (diff < 0) ? -diff : diff;
		if (diff < bestDiff)
		{
			bestDiff = diff;
			bestIdx = idx;	// 繧医ｊ霑代＞繝槭・繧ｫ繝ｼ繧呈治逕ｨ
		}
	}
	return bestIdx;
}
/////////////////////////////////////////////////////////////////////
// 繝ｭ繝ｼ繧ｫ繝ｫ髢｢謨ｰ activateFailSafe
// 蜃ｦ逅・ｦりｦ・ 陬懈ｭ｣螟ｱ謨玲凾縺ｮ繝輔ぉ繧､繝ｫ繧ｻ繝ｼ繝暮溷ｺｦ蛻ｶ髯舌ｒ驕ｩ逕ｨ縺吶ｋ
// 蠑墓焚	 縺ｪ縺・
// 謌ｻ繧雁､	 縺ｪ縺・
/////////////////////////////////////////////////////////////////////
static void activateFailSafe(void)
{
	if (failSafeActive)
	{
		return;	// 譌｢縺ｫ逋ｺ蜍墓ｸ医∩縺ｪ繧我ｽ輔ｂ縺励↑縺・
	}
	float currentSpeed = (float)targetSpeed / PULSE_MILLIMETER;	// 迴ｾ蝨ｨ縺ｮ逶ｮ讓咎溷ｺｦ[m/s]
	float limitedSpeed = currentSpeed * FAILSAFE_SPEED_SCALE;	// 謖・ｮ壼咲紫縺ｧ螳牙・蛛ｴ縺ｫ貂幃・
	setTargetSpeed(limitedSpeed);	// 騾溷ｺｦ謖・ｻ､繧呈峩譁ｰ
	boostSpeed = limitedSpeed;	// 蜿ら・騾溷ｺｦ繧ょ酔譛・
	failSafeActive = true;
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・processMarkerEvent
// 蜃ｦ逅・ｦりｦ・    繝槭・繧ｫ繝ｼ騾夐℃譎ゅ・蜃ｦ逅・ｒ縺ｾ縺ｨ繧√ｋ
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void processMarkerEvent(void) {
	// 繧ｫ繝ｼ繝悶・繝ｼ繧ｫ繝ｼ,繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ繧呈､懷・縺励◆譎ゅ・蜃ｦ逅・
	if (courseMarker > 0 && beforeCourseMarker == 0) {
		cntMarker++; // 繝槭・繧ｫ繝ｼ繧ｫ繧ｦ繝ｳ繝・
		if (optimalTrace == BOOST_DISTANCE) {
			if (numPPAMarry > 0) {
				bool isCross = (courseMarker == CROSSLINE);	// 繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｪ繧臥┌譚｡莉ｶ陬懈ｭ｣
				bool straightLike = isStraightBeforeMarker(encTotalOptimal, STRAIGHT_WINDOW_MM, STRAIGHT_RATIO_THRESHOLD);	// 逶ｴ邱夂紫蛻､螳・
				bool straightPending = straightMarkerPending;	// first marker after straight detection
				bool usedStraightPending = straightPending;
				if (straightLike || isCross || straightPending) {
					if (straightPending) {
						straightMarkerPending = false;
					}
					int16_t nearestIdx = findNearestMarkerIndex(encTotalOptimal, isCross);	// 霑大ｍ縺九ｉ譛驕ｩ繝槭・繧ｫ繝ｼ繧貞叙蠕・
					int32_t rawDiff = encTotalOptimal - markerPos[nearestIdx].distance;	// 迴ｾ蝨ｨ霍晞屬縺ｨ縺ｮ蟾ｮ蛻・繝代Ν繧ｹ]
					int32_t absDiff = (rawDiff < 0) ? -rawDiff : rawDiff;
					int32_t allowDiff = calcDynamicThresholdPulse();	// 蜍慕噪縺ｫ邂怜・縺励◆險ｱ螳ｹ隱､蟾ｮ
					bool canCorrect = isCross || (absDiff <= allowDiff);	// 繧ｯ繝ｭ繧ｹ縺ｯ蜊ｳ陬懈ｭ｣縲√◎繧御ｻ･螟悶・髢ｾ蛟､蛻､螳・
					pathedMarker = clampMarkerIndex(nearestIdx);	// 繝偵Φ繝井ｽ咲ｽｮ繧呈峩譁ｰ
					if (canCorrect) {
						if (usedStraightPending) {
							straightMarkerPendingLog = 1;
						}
						int32_t stepLimit = encMM(CORR_STEP_MAX_MM);	// 谿ｵ髫手｣懈ｭ｣縺ｮ荳企剞驥充繝代Ν繧ｹ]
						int32_t diff = rawDiff;
						if (diff > stepLimit) {
							diff = stepLimit;	// 谿ｵ髫手｣懈ｭ｣縺ｧ蛻・ｊ隧ｰ繧・
						}
						if (diff < -stepLimit) {
							diff = -stepLimit;
						}
						int32_t errorDistance = encTotalOptimal - DistanceOptimal;	// 陬懈ｭ｣蜑阪・霍晞屬隱､蟾ｮ繧剃ｿ晄戟
						Control_ApplyMarkerCorrection_p(diff);	// 繝槭・繧ｫ繝ｼ陬懈ｭ｣繧偵せ繝ｪ繝・・陬懈ｭ｣蠕後ヱ繝ｫ繧ｹ縺ｸ蜿肴丐
						DistanceOptimal = encTotalOptimal - errorDistance;	// 隱､蟾ｮ繧堤ｶｭ謖√＠縺溘∪縺ｾ逶ｮ讓呵ｷ晞屬繧呈峩譁ｰ
						int32_t markerIndex = markerPos[nearestIdx].indexPPAD;	// PPAD蛛ｴ縺ｮ蟇ｾ蠢懊う繝ｳ繝・ャ繧ｯ繧ｹ
						int32_t currentIndex = (int32_t)optimalIndex;
						int32_t nearDev = isCross ? MARKER_INDEX_DEV_CROSS : MARKER_INDEX_DEV_NORMAL;
						// marker index繧堤樟蝨ｨ縺ｮoptimalIndex霑大ｍ縺ｫ諡俶據縺吶ｋ
						if (markerIndex > currentIndex + nearDev)
						{
							markerIndex = currentIndex + nearDev;
						}
						if (markerIndex < currentIndex - nearDev)
						{
							markerIndex = currentIndex - nearDev;
						}
						// 繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ陬懈ｭ｣譎ゅ・1蝗槭〒縺ｮ繧ｸ繝｣繝ｳ繝鈴㍼繧偵＆繧峨↓蛻ｶ髯舌☆繧・
						if (isCross)
						{
							if (markerIndex > currentIndex + MARKER_INDEX_JUMP_CROSS_MAX)
							{
								markerIndex = currentIndex + MARKER_INDEX_JUMP_CROSS_MAX;
							}
							if (markerIndex < currentIndex - MARKER_INDEX_JUMP_CROSS_MAX)
							{
								markerIndex = currentIndex - MARKER_INDEX_JUMP_CROSS_MAX;
							}
						}
						if (markerIndex >= 0 && markerIndex < numPPADarry) {
							optimalIndex = (uint16_t)markerIndex;
						} else if (numPPADarry > 0) {
							optimalIndex = (uint16_t)(numPPADarry - 1);
						} else {
							optimalIndex = 0;
						}
						boostSpeed = PPAD[optimalIndex].boostSpeed;	// 蛹ｺ髢馴溷ｺｦ繧貞叙蠕・
						setTargetSpeed(boostSpeed);	// 逶ｮ讓咎溷ｺｦ縺ｸ蜊ｳ蜿肴丐
						resetSpeedPID();	// PID蜀・Κ迥ｶ諷九ｒ蜷梧悄
						int16_t newPathed = nearestIdx - 2;	// 谺｡蝗樊爾邏｢縺ｯ蟆代＠謇句燕縺九ｉ
						pathedMarker = clampMarkerIndex(newPathed);
						lastCorrectedMarker = nearestIdx;	// 逶ｴ霑題｣懈ｭ｣菴咲ｽｮ繧定ｨ倬鹸
						missedCorrections = 0;	// 螟ｱ謨励き繧ｦ繝ｳ繧ｿ繧偵Μ繧ｻ繝・ヨ
						failSafeActive = false;	// 繝輔ぉ繧､繝ｫ繧ｻ繝ｼ繝戊ｧ｣髯､
					} else {
						missedCorrections++;	// 陬懈ｭ｣螟ｱ謨励ｒ繧ｫ繧ｦ繝ｳ繝・
						if (missedCorrections >= FAILSAFE_MISS_MAX) {
							activateFailSafe();
						}
					}
				} else {
					missedCorrections++;	// 逶ｴ邱壽擅莉ｶ荳肴・遶九〒繧ょ､ｱ謨玲桶縺・
					if (missedCorrections >= FAILSAFE_MISS_MAX) {
						activateFailSafe();
					}
				}
			}
		} else if(optimalTrace == BOOST_SHORTCUT) {
			// 繧ｷ繝ｧ繝ｼ繝医き繝・ヨ蝓ｺ貅・谺｡襍ｰ陦後・縺ｨ縺・
		}
	}
	beforeCourseMarker = courseMarker; // 蜑榊屓縺ｮ繝槭・繧ｫ繝ｼ迥ｶ諷九ｒ譖ｴ譁ｰ
}
/////////////////////////////////////////////////////////////////////
// 繝｢繧ｸ繝･繝ｼ繝ｫ蜷・cleaerMarkerProcessState
// 蜃ｦ逅・ｦりｦ・    繝槭・繧ｫ繝ｼ騾夐℃蜃ｦ逅・憾諷九・蛻晄悄蛹・
// 蠑墓焚         縺ｪ縺・
// 謌ｻ繧雁､       縺ｪ縺・
/////////////////////////////////////////////////////////////////////
void clearMarkerProcessState(void) {
	beforeCourseMarker = 0;
	cntMarker = 0;
	straightMeter = 0;	// 霑ｽ蜉: 逶ｴ邱壼愛螳夊ｷ晞屬繧貞・譛溷喧
	straightState = false;
	straightMarkerPending = false;
	straightMarkerPendingLog = 0;
	pathedMarker = 0;
	lastCorrectedMarker = 0;
	missedCorrections = 0;
	failSafeActive = false;
}

