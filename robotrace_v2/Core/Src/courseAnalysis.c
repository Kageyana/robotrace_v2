//====================================//
// インクルード
//====================================//
#include "courseAnalysis.h"
#include "courseLogCsv.h"
#include "control.h"
#include "fatfs.h"
#include "PIDcontrol.h"
#include "markerSensor.h"
#include "BMI088.h"
#include "SDcard.h"
#include "pathFollower.h"
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
// グローバル変数の宣言
//====================================//
uint8_t optimalTrace = 0;
uint16_t optimalIndex;
int16_t numPPADarry; // path palanning analysis distance (PPAD)
int16_t numPPAMarry; // path palanning analysis marker (PPAM1)
int16_t indexSC;
int16_t pathedMarker = 0;
float boostSpeed;
int32_t DistanceOptimal = 0; // 2次走行用の目標走行距離[pulse]
int16_t analyzedNumber = 0;	 // 前回解析したログ番号
int32_t encTotalOptimal = 0; // 2次走行用の補正済み走行距離[pulse]
int32_t encPID = 0;			 // 距離制御用の距離[pulse]
float xydegz = 0;
static int32_t xyPreviousTotalPulse = 0;
int32_t straightMeter;
bool straightState;
bool straightMarkerPending;
uint8_t straightMarkerPendingLog;

static uint8_t missedCorrections = 0;	// 連続補正失敗回数
static bool failSafeActive = false;	// フェイルセーフ動作中フラグ
static int16_t lastCorrectedMarker = 0;	// 直前に補正したマーカーインデックス

static void logReadSlipIoError(int logNumber, UINT lineNo, FIL *fil, const char *tag);
static float calcDecelLeadMmByRoc(int16_t rocPrev, int16_t rocNow);
static void applyDecelLeadToPpad(int16_t count);
static void applyDecelLeadToArray(float *speed, int16_t count);

typedef CourseLogColumnMap SecondLogColumnMap;

static bool parseSecondLogHeader(const char *line, SecondLogColumnMap *map)
{
	return courseLogParseHeaderLine(line, map) && courseLogHasSlipColumns(map);
}

AnalysisData PPAD[OPT_BUFF_SIZE];
EventPos markerPos[OPT_BUFF_SIZE];
Courseplot xycie;							   // XY座標値（走行中に計算し、ログ保存に使用する）

/////////////////////////////////////////////////////////////////////
// モジュール名 calcROC
// 処理概要     速度と角速度から曲率半径を計算する
// 引数         velo: エンコーダ速度[pulse/ms], angvelo: 角速度[deg/s], dt: 積分時間[s]
// 戻り値       曲率半径[mm]
/////////////////////////////////////////////////////////////////////
float calcROC(int16_t velo, float angvelo, float dt)
{
	// エンコーダ速度と積分時間から移動距離[mm]を計算する
    float dl = calcDlMm(velo, dt);  // [mm]
    // 角速度[deg/s]をrad/sへ変換し、dt[s]を掛けて角度変化量[rad]を求める
    float drad = angvelo * DEG2RAD * dt;

    // 絶対値を条件式で求める。
    float absDrad = (drad < 0.0f) ? -drad : drad;
    float absDl   = (dl   < 0.0f) ? -dl   : dl;

    // 直線判定: |dl/drad| > ROC_STRAIGHT_TH を乗算による比較に置き換える
    // 除算せずに曲率半径の閾値と比較する
    if (absDrad < 1e-6f || ROC_STRAIGHT_TH * absDrad < absDl) {
        return ROC_STRAIGHT_MAX; // 直線とみなす
    }

    // カーブの場合のみ除算する
    float R = absDl / absDrad;
    // float absR = (R < 0.0f) ? -R : R;
    // return (absR > ROC_STRAIGHT_TH) ? 2000.0f : R;

	return R;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 saveLogNumber
// 処理概要     解析したログファイルの番号を設定ファイルに保存する
// 引数         fileNumber: 保存するログ番号
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void saveLogNumber(int16_t fileNumber)
{
	FRESULT fresult;
	FIL fil;
	char fileName[32] = PATH_SETTING;

	strcat(fileName, FILENAME_ANALYSIS_NUMBER);					 // ファイル名を追加する
	strcat(fileName, ".txt");									 // 拡張子を追加する
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
// モジュール名 getLogNumber
// 処理概要     設定ファイルから解析済みログ番号を取得する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getLogNumber(void)
{
	FRESULT fresult;
	FIL fil;
	TCHAR log[20];
	char fileName[32] = PATH_SETTING;
	int parsedNumber = analyzedNumber;
	bool repair = false;

	strcat(fileName, FILENAME_ANALYSIS_NUMBER);					// ファイル名を追加する
	strcat(fileName, ".txt");									// 拡張子を追加する
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // 解析済みログ番号の設定ファイルを開く
	if (fresult == FR_OK)
	{
		// 解析済みログ番号を取得する
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
		// 解析済みログ番号に一致するファイル一覧のインデックスを保存する
		if (analyzedNumber == fileNumbers[i])
		{
			fileIndexLog = i;
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 sortInt16Ascending
// 処理概要     int16_t配列を挿入ソートし、少数要素のqsort呼び出しを省く
// 引数         values: ソート対象の配列, length: 要素数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void sortInt16Ascending(int16_t *values, uint16_t length)
{
	if (length <= 1)
	{
		return; // 要素数が1以下なら並べ替え不要
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
// モジュール名 calcDecelLeadMmByRoc
// 処理概要     曲率半径の変化率から先行減速距離を計算する
// 引数         rocPrev: 直前の曲率半径[mm], rocNow: 現在の曲率半径[mm]
// 戻り値       先行減速距離[mm]
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
		return 0.0f; // 曲率半径が大きくなる方向では先行減速しない
	}
	if (absPrev <= 0)
	{
		absPrev = 1;
	}

	float changeRatio = (float)(absPrev - absNow) / (float)absPrev; // 曲率半径の減少率（0.0～1.0）
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
// モジュール名 applyDecelLeadToPpad
// 処理概要     PPAD速度配列の減速区間に先行減速を適用する
// 引数         count: PPAD配列の有効要素数
// 戻り値       なし
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

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // 先行減速距離[mm]を切り上げて配列ステップ数へ換算する
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
			// 減速後の速度を手前の区間にも適用する
			if (PPAD[j].boostSpeed > nowSpeed)
			{
				PPAD[j].boostSpeed = nowSpeed;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 applyDecelLeadToArray
// 処理概要     指定した速度配列の減速区間に先行減速を適用する
// 引数         speed: 速度配列[m/s], count: 有効要素数
// 戻り値       なし
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

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // 先行減速距離[mm]を切り上げて配列ステップ数へ換算する
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
			// 減速後の速度を手前の区間にも適用する
			if (speed[j] > nowSpeed)
			{
				speed[j] = nowSpeed;
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readLogDistanceProgress
// 処理概要     一次ログから距離基準の速度計画を生成する
// 引数         logNumber: 一次ログ番号、progress: 解析進捗の通知関数
// 戻り値       計画の要素数、失敗時は負のエラーコード
/////////////////////////////////////////////////////////////////////
static int16_t readLogDistanceProgress(int logNumber,
	void (*progress)(const char *stage, uint32_t lineNo))
{
	// ログファイルを読み込むための変数
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool fileOpened = false; // f_close
	bool retried = false;
	bool errorDetected = false; // 解析途中のエラー発生を示すフラグ
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9;
	}
	// 解析中はログ書き込みを抑制する
	sd_set_analysis_active(true); // SD/FatFs使用中
	snprintf(fileName, sizeof(fileName), "%d", logNumber); // ログ番号をファイル名に変換する
	strcat(fileName, ".csv"); // CSV拡張子を追加する
	retry_open:
	// ログ保存時に同期・クローズ済みのファイルを開く。I/Oエラー時の再試行だけ再マウントする。
	if (retried && !sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup_read;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // CSVファイルを読み取り専用で開く
	if (!retried && (fresult == FR_DISK_ERR || fresult == FR_INT_ERR || fresult == FR_NOT_READY))
	{
		retried = true;
		goto retry_open;
	}

	if (fresult == FR_OK)
	{
		fileOpened = true; // 正常に開けたファイルだけを後で閉じる
		// ログデータを取得する
	static TCHAR log[4096];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t marker, distance, roc;
		int32_t numD = 0, numM = 0, cntCurR = 0, numStraight = 0;
		int32_t previousDistance = 0, analysisBaseDistance = 0;
		bool havePreviousDistance = false;
		static int16_t ROCbuff[600] = {0};
		int16_t sortROC[(CALCDISTANCE / LOG_DISTANCE_MM) + 2U];
		int32_t straightMeter = 0;
		bool straightState = false;

		// 解析前にデータ配列を初期化する
		// PPAD配列をゼロクリアする
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		CourseLogColumnMap distanceColumns;
		TCHAR *header = f_gets(log, log_len, &fil_Read);
		if (!header)
		{
			ret = f_error(&fil_Read) ? -5 : -2;
			errorDetected = true;
			if (ret == -5) logReadSlipIoError(logNumber, 0, &fil_Read, "io_fail");
		}
		else if (!courseLogResolveHeader((const char *)header, NULL, false, &distanceColumns))
		{
			TCHAR *secondHeader = f_gets(log, log_len, &fil_Read);
			if (!secondHeader || !courseLogResolveHeader((const char *)header,
				(const char *)secondHeader, false, &distanceColumns))
			{
				ret = -2;
				errorDetected = true;
			}
		}

		UINT lineNo = 0;
		if (progress) progress("Base read", 0U);
		// ログデータの読み込みを開始する
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
			if (progress && (lineNo % 512U) == 0U) progress("Base read", lineNo);

			CourseLogDistanceRow row;
			if (!courseLogParseDistanceRow((const char *)log, &distanceColumns, &row))
			{
				continue;
			}
			marker = row.courseMarker;
			distance = row.encTotalOptimal;
			roc = (int32_t)lroundf(row.ROC);
			if (!havePreviousDistance)
			{
				previousDistance = distance;
				analysisBaseDistance = distance;
				havePreviousDistance = true;
			}
			// マーカー状態を解析する
			// marker==3は交差ラインのマーカー
			// marker==2は左マーカー。直線走行中だけカーブマーカーとして扱う
			if (marker == 3 || (marker == 2 && straightState))
			{
				// カーブマーカー通過時に位置を記録する
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;

				if (marker == 2 && straightState)
				{
				// 直線後の左マーカーを検出したら直線状態と距離をリセットする
					straightState = false;
					straightMeter = 0;
				}

				numM++; // マーカー解析インデックスを更新する
			}

			// 行間隔の変動を考慮し、実距離50 mmごとに解析する。
			if (distance - analysisBaseDistance >= encMM(CALCDISTANCE))
			{
				int32_t copyCount = cntCurR;
				if (copyCount > (int32_t)(sizeof(sortROC) / sizeof(sortROC[0])))
				{
					copyCount = (int32_t)(sizeof(sortROC) / sizeof(sortROC[0]));
				}
				for (int32_t sortIndex = 0; sortIndex < copyCount; sortIndex++)
				{
					sortROC[sortIndex] = ROCbuff[sortIndex]; // 中央値計算に必要な範囲をコピーする
				}
				sortInt16Ascending(sortROC, (uint16_t)copyCount); // 曲率半径の中央値算出用に昇順ソートする

				// 曲率半径の中央値を求める
				if (copyCount % 2 == 0)
				{
					// サンプル数が偶数なら中央2値の平均を使う
					PPAD[numD].ROC = (sortROC[copyCount / 2] + sortROC[copyCount / 2 - 1]) / 2;
				}
				else
				{
					// サンプル数が奇数なら中央の値を使う
					PPAD[numD].ROC = sortROC[copyCount / 2];
				}

				PPAD[numD].boostSpeed = asignVelocity(PPAD[numD].ROC); // 曲率半径から目標速度を計算する

				// 前区間と同じ曲率半径なら直線区間数を加算する
				if (numD >= 1 && PPAD[numD].ROC == PPAD[numD - 1].ROC)
				{
					numStraight++;
				}
				else
				{
					numStraight = 0;
				}

				cntCurR = 0; // 曲率半径用サンプル数をクリアする
				numD++; // 距離解析インデックスを更新する
				analysisBaseDistance += encMM(CALCDISTANCE);
				if (numD >= OPT_BUFF_SIZE)
				{
					ret = -1; // 解析配列の容量超過をエラーとして返す
					errorDetected = true; // 共通クリーンアップへ移る
					break; // 解析ループを終了する
				}
			}
			// 曲率判定と直線距離を、直前ログ行からの距離差で更新する。
			int32_t distanceStep_p = distance - previousDistance;
			if (abs(roc) >= 700)
			{
				if (distanceStep_p > 0)
				{
					straightMeter += (int32_t)lroundf((float)distanceStep_p / PULSE_MILLIMETER);
				}
			}
			else
			{
				straightMeter = 0;
			}

			// 直線距離が100 mm以上続いたら直線走行中と判定する。
			// 次に検出する左マーカーをカーブ開始位置として扱う。
			if (straightMeter >= 100)
			{
				straightState = true;
			}

			if (cntCurR < (int32_t)(sizeof(sortROC) / sizeof(sortROC[0])))
			{
				ROCbuff[cntCurR] = (int16_t)roc;
				cntCurR++;
			}
			previousDistance = distance;
		}

		if (!errorDetected)
		{
			if (progress) progress("Base plan", lineNo);
			// 要素数を末尾インデックスへ調整する
			if (numM > 0)
			{
				numM--;        // マーカーがある場合だけ減算し、負値を防ぐ
			}
			int32_t numDCount = 0;
			if (numD > 0)
			{
				numD--;        // 距離要素がある場合だけ減算し、負値を防ぐ
				numDCount = numD + 1;        // 要素数に戻して加減速調整で使用する
			}
			else
			{
				numDCount = numD;        // 要素数が0ならそのまま使用する
			}
			numD = numDCount;
			applyDecelLeadToPpad((int16_t)numD); // 曲率半径の変化に応じて減速開始位置を手前へ移す

			// 目標速度配列を整形し、加減速を制限する
			float acceleration, elapsedTime, dv, dl;

			// 解析区間長をmmからmへ変換する
			dl = (float)CALCDISTANCE / 1000;

			// numDを要素数として扱い、以下のループを有効範囲内に限定する

			// インデックス1から末尾へ進み、加速側の速度変化を制限する
			for (int32_t idx = 1; idx < numD; idx++)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx - 1].boostSpeed);	// 区間速度差[m/s]
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 速度差が極小なら補正不要
				}
				elapsedTime = fabsf(dl / dv);		// 区間時間[s]
				acceleration = dv / elapsedTime;	// 区間の速度差から計算した加速度[m/s^2]
				if (acceleration > MACHINEACCELE)
				{
					PPAD[idx].boostSpeed = PPAD[idx - 1].boostSpeed + (MACHINEACCELE * dl);
				}
			}

			// 末尾から先頭へ戻り、減速側の速度変化を制限する
			for (int32_t idx = numD - 2; idx >= 0; idx--)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx + 1].boostSpeed);	// 区間速度差[m/s]
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 速度差が極小なら補正不要
				}
				elapsedTime = fabsf(dl / dv);
				acceleration = dv / elapsedTime;
				if (acceleration > MACHINEDECREACE)
				{
					PPAD[idx].boostSpeed = PPAD[idx + 1].boostSpeed + (MACHINEDECREACE * dl);
				}
			}

#ifdef WRITE_BOOSTSPEED_LOG
			// 整形後の目標速度配列をSDカードへ記録する
			FIL fil_Boost;
			FRESULT fresult_Boost;
			char boostFileName[32];
			snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
			fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
			if (fresult_Boost == FR_OK)
			{
				// CSVヘッダを書き込み、整形済みのboostSpeedを順番に保存する
				UINT bytesWritten;
				f_printf(&fil_Boost, "index,boost_speed\n");
				for (int32_t idx = 0; idx < numD; idx++)
				{
					char boostLine[48];

					// f_printfは%f非対応のため、1行分を文字列に整形してから書き込む
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
			// エラー発生時は整形処理を行わず、解析結果を採用しない
		}
	}
	else
	{
		ret = -4;
		errorDetected = true; // ファイルを開けなかった場合もエラーとして扱う
	}

	if (ret == -5 && !retried)
	{
		// I/Oエラー時は一度だけ再マウントしてファイルを開き直す
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
		f_close(&fil_Read); // 正常に開けたファイルだけを閉じる
	}
	if (lock_acquired)
	{
		// 解析終了時に書き込み抑制とロックを解除する
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	// printf("Analysis distance end\n");

	if (ret >= 0)
	{
		// 正常終了時だけ解析済み情報を更新する
		saveLogNumber(logNumber);
		analyzedNumber = logNumber;

		// 距離基準2次走行モードを設定する
		optimalTrace = BOOST_DISTANCE;
	}
	else
	{
		// エラー発生時は状態を更新せず、呼び出し元へエラーを返す
	}

	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readLogDistance
// 処理概要     一次ログから距離基準の速度計画を生成する
// 引数         logNumber: 一次ログ番号
// 戻り値       計画の要素数、失敗時は負のエラーコード
/////////////////////////////////////////////////////////////////////
int16_t readLogDistance(int logNumber)
{
	return readLogDistanceProgress(logNumber, NULL);
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 parseSecondLogLine
// 処理概要     2次走行ログからスリップ解析に必要な列を抽出する
// 引数         line: CSV行, map: 列対応表, 各出力ポインタ: 抽出結果の格納先
// 戻り値       true: 解析成功, false: 解析失敗または値が範囲外
/////////////////////////////////////////////////////////////////////
static bool parseSecondLogLine(const char *line, const SecondLogColumnMap *map,
		uint8_t *courseMarker, int32_t *encTotal, int16_t *roc,
		float *targetSpeedLog, int16_t *optimalIdx, uint8_t *slipLong, uint8_t *slipLat)
{
	CourseLogSlipRow row;
	if (!courseLogParseSlipRow(line, map, &row) ||
		row.courseMarker < 0 || row.courseMarker > UINT8_MAX ||
		row.optimalIndex < INT16_MIN || row.optimalIndex > INT16_MAX ||
		row.slipFlag < 0 || row.slipFlag > UINT8_MAX ||
		row.slipFlagLat < 0 || row.slipFlagLat > UINT8_MAX)
	{
		return false;
	}
	*courseMarker = (uint8_t)row.courseMarker;
	*encTotal = row.encTotalOptimal;
	*roc = (int16_t)lroundf(row.ROC);
	*targetSpeedLog = row.targetSpeed;
	*optimalIdx = (int16_t)row.optimalIndex;
	*slipLong = (uint8_t)row.slipFlag;
	*slipLat = (uint8_t)row.slipFlagLat;
	return true;
}
static void logReadSlipIoError(int logNumber, UINT lineNo, FIL *fil, const char *tag)
{
	DWORD pos = f_tell(fil);
	DWORD size = f_size(fil);
	int eof = f_eof(fil);
	int err = f_error(fil);
	// 読み取りエラー中のSDカードに診断を書き込まない。
	printf("readLogDistanceSlip log=%d %s: line=%lu pos=%lu size=%lu eof=%d err=%d sd_sector=%lu sd_count=%u sd_rb=%d sd_rm=%d\n",
		logNumber, (tag != NULL) ? tag : "io",
		(unsigned long)lineNo, (unsigned long)pos, (unsigned long)size, eof, err,
		(unsigned long)g_sd_last_read_sector, (unsigned int)g_sd_last_read_count,
		g_sd_last_read_blocks_status, g_sd_last_read_multi_status);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readLogDistanceSlip
// 処理概要     一次ログの距離計画に直前走行のスリップを反映する
// 引数         baseLogNumber: 一次ログ番号、slipLogNumber: 直前走行ログ番号
//              progress: 解析段階と読込行数を通知する関数
// 戻り値       計画の要素数、失敗時は負のエラーコード
/////////////////////////////////////////////////////////////////////
int16_t readLogDistanceSlip(int16_t baseLogNumber, int16_t slipLogNumber,
	void (*progress)(const char *stage, uint32_t lineNo))
{
	if (baseLogNumber <= 0 || slipLogNumber <= 0)
	{
		return -4;
	}
	// 直前DISTANCE走行の計画が同じ一次ログを元にしていれば再利用する。
	// 計画が無い場合だけ一次ログを読み直す。
	if (optimalTrace == BOOST_DISTANCE && analyzedNumber == baseLogNumber &&
		numPPADarry > 0 && numPPADarry <= OPT_BUFF_SIZE)
	{
		if (progress) progress("Base cached", (uint32_t)numPPADarry);
	}
	else
	{
		if (progress) progress("Base", 0U);
		int16_t baseRet = readLogDistanceProgress(baseLogNumber, progress);
		if (baseRet < 0)
		{
			return baseRet;
		}
	}
	int16_t baseCount = numPPADarry;
	if (baseCount <= 0)
	{
		return -2; // 解析対象がない
	}
	if (progress) progress("Slip open", 0U);

	FIL fil_Read;
	FRESULT fresult;
	char fileName[16];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);
	bool fileOpened = false;
	bool retried = false;

	if (!lock_acquired)
	{
		return -9; // SD/FatFs使用中
	}
	// 解析中はログ書き込みを抑制する
	sd_set_analysis_active(true);

	// 解析用配列を静的領域に確保し、スタック使用量を抑える
	static uint16_t sampleCnt[OPT_BUFF_SIZE];
	static float v2Max[OPT_BUFF_SIZE];
	static float rocAbsSum[OPT_BUFF_SIZE];
	static uint16_t rocCnt[OPT_BUFF_SIZE];
	static uint16_t slipLongCnt[OPT_BUFF_SIZE];
	static uint16_t slipLatCnt[OPT_BUFF_SIZE];
	static float risk[OPT_BUFF_SIZE];
	static float riskExpanded[OPT_BUFF_SIZE];
	static float v3[OPT_BUFF_SIZE];

	snprintf(fileName, sizeof(fileName), "%d", slipLogNumber);			   // ログ番号を文字列へ変換する
	strcat(fileName, ".csv");										   // CSV拡張子を追加する
	retry_open_slip:
	if (retried && progress) progress("Slip retry", 0U);
	// 一次ログ解析後も同じマウントを使い、I/Oエラー後だけ再マウントする。
	if (retried && !sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // CSVファイルを読み取り専用で開く
	if (!retried && (fresult == FR_DISK_ERR || fresult == FR_INT_ERR || fresult == FR_NOT_READY))
	{
		retried = true;
		goto retry_open_slip;
	}
	if (fresult != FR_OK)
	{
		ret = -4; // ログファイルを開けなかった
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

	// 先頭行を読み、列名行またはメタデータ行として扱う
	TCHAR *header = f_gets(log, log_len, &fil_Read);
	if (!header)
	{
		int eof = f_eof(&fil_Read);
		int err = f_error(&fil_Read);
		if (!eof && err != 0)
		{
			ret = -5; // f_gets I/O error
			logReadSlipIoError(slipLogNumber, 0, &fil_Read, "io_fail");
		}
		else
		{
			ret = -2; // 解析対象がない
		}
		goto cleanup;
	}
	if (!parseSecondLogHeader((const char *)header, &secondLogColumns) &&
		(f_gets(log, log_len, &fil_Read) == NULL ||
		 !parseSecondLogHeader((const char *)log, &secondLogColumns)))
	{
		ret = -2; // 2次ログに必要な列がない
		goto cleanup;
	}

	UINT lineNo = 0;
	bool fgets_null = false;
	if (progress) progress("Slip read", 0U);

	while (1) {
		TCHAR* s = f_gets(log, log_len, &fil_Read);
		if (!s)
		{
			fgets_null = true;
			break;
		}
		lineNo++;
		if (progress && (lineNo % 512U) == 0U) progress("Slip read", lineNo);

		if (f_error(&fil_Read))
		{
			logReadSlipIoError(slipLogNumber, lineNo, &fil_Read, "io_err");
			ret = -5;
			break;
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
			continue;	// 解析できない行を読み飛ばす
		}
		if (optimalIdx < 0 || optimalIdx >= baseCount)
		{
			ret = -1;	// 解析用配列の有効範囲外
			break;
		}

		// 区間ごとのサンプル数、曲率半径の絶対値、スリップ回数を集計する
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

		// 読み取った区間までの集計を継続する
	}
	if (ret == 0 && fgets_null)
	{
		int eof = f_eof(&fil_Read);
		int err = f_error(&fil_Read);
		if (!eof && err != 0)
		{
			ret = -5; // f_gets I/O error
			logReadSlipIoError(slipLogNumber, lineNo, &fil_Read, "io_fail");
		}
	}

	if (ret == -5 && !retried)
	{
		// I/Oエラー時は一度だけ再マウントしてファイルを開き直す
		if (fileOpened)
		{
			f_close(&fil_Read);
			fileOpened = false;
		}
		retried = true;
		ret = 0;
		goto retry_open_slip;
	}
	if (progress) progress("Slip plan", lineNo);

cleanup:
	if (fileOpened)
	{
		f_close(&fil_Read);
	}
	if (lock_acquired)
	{
		// 解析終了時に書き込み抑制とロックを解除する
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	if (ret < 0)
	{
		return ret;
	}
	if (maxOptimalIndex < 0)
	{
		return -2;	// 解析対象がない
	}

	// サンプルのない区間の曲率半径集計を直前の値で補う
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		if (sampleCnt[i] == 0 && i > 0)
		{
			rocAbsSum[i] = rocAbsSum[i - 1];	// 平均曲率半径の計算用に直前の合計値を引き継ぐ
			rocCnt[i] = rocCnt[i - 1];			// 平均曲率半径の計算用に直前の件数を引き継ぐ
		}
	}

	// スリップ割合からリスク値（0.0～1.0）を求める
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

	// 近傍区間へスリップリスクを拡張する
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

	// スリップリスクに応じて速度計画v3を減速または増速する
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

	// 前後方向の走査で、元の速度計画の速度変化量以内に制限する
	applyDecelLeadToArray(v3, (int16_t)(maxOptimalIndex + 1)); // 曲率半径の変化に応じて減速開始位置を手前へ移す
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

	// 更新した速度計画をPPADへ反映する
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		PPAD[i].boostSpeed = v3[i];
	}

	ret = numPPADarry;

#ifdef WRITE_BOOSTSPEED_LOG
	// 整形後の目標速度配列をSDカードへ記録する
	FIL fil_Boost;
	FRESULT fresult_Boost;
	char boostFileName[32];
	snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, slipLogNumber);
	fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
	if (fresult_Boost == FR_OK)
	{
		// CSVヘッダを書き込み、整形済みのboostSpeedを順番に保存する
		UINT bytesWritten;
	f_printf(&fil_Boost, "index,boost_speed\n");
	for (int32_t idx = 0; idx < maxOptimalIndex; idx++)
	{
		char boostLine[48];

			// f_printfは%f非対応のため、1行分を文字列に整形してから書き込む
			snprintf(boostLine, sizeof(boostLine), "%ld,%.3f\n", (long)idx, PPAD[idx].boostSpeed);
			f_write(&fil_Boost, boostLine, strlen(boostLine), &bytesWritten);
		}
		f_close(&fil_Boost);
	}
#endif

	// 距離基準2次走行モードを設定する
	optimalTrace = BOOST_DISTANCE;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 asignVelocity
// 処理概要     曲率半径に応じた目標速度を割り当てる
// 引数         ROC: 曲率半径[mm]
// 戻り値       目標速度[m/s]
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
// モジュール名 cmpfloat
// 処理概要     float値を比較する
// 引数         n1: 比較する第1の値へのポインタ, n2: 第2の値へのポインタ
// 戻り値       n1が大きければ1、小さければ-1、同値なら0
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
// モジュール名 readLogTest
// 処理概要     試験用ログを読み込み、マーカー位置と読込件数を求める
// 引数         logNumber: 読み込むログ番号
// 戻り値       読込データ件数、失敗時は負のエラーコード
/////////////////////////////////////////////////////////////////////
int16_t readLogTest(int logNumber)
{
	// ログファイルを読み込むための変数
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9; // SD/FatFs使用中
	}

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // ログ番号を文字列へ変換する
	strcat(fileName, ".csv");										   // CSV拡張子を追加する
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // CSVファイルを読み取り専用で開く

	if (fresult == FR_OK)
	{
		static TCHAR log[4096];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t time, marker, velo, distance;
		float angVelo;
		int32_t startEnc = 0, numD = 0, numM = 0, beforeMarker = 0;
		bool analysis = false;

		// 解析の前処理
		// 解析データ配列を初期化する
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		// ログデータの読み込みを開始する
		while (f_gets(log, log_len, &fil_Read))
		{
			if (sscanf(log, "%ld,%ld,%f,%ld,%ld", &time, &velo, &angVelo, &marker, &distance) != 5)
			{
				continue; // メタデータ行と列名行を除外
			}

			// マーカー状態を解析する
			if (marker == 1 && beforeMarker == 0)
			{
				// ゴールマーカー検出時に解析フラグを反転する
				analysis = !analysis;
				startEnc = distance;
			}
			else if (marker == 0 && beforeMarker == 2)
			{
				// カーブマーカー通過時にマーカー位置を記録する
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;
				numM++; // マーカー解析インデックスを更新する
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

	// 解析済みログ番号を更新する
	// saveLogNumber(logNumber);
	analyzedNumber = logNumber;

	return ret;
}
// モジュール名 calcXYcies
// 処理概要     一次走行ログから再走行またはショートカット経路を生成する
// 引数         logNumber: 解析するログ番号
// 戻り値       経路点数、負値はエラー
/////////////////////////////////////////////////////////////////////
int16_t calcXYcies(int logNumber)
{
	return routeBuildFromLog(logNumber, shortcutSettings.maxLevel);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcXYcie
// 処理概要     累積距離と1ms積算角度からログ用XY座標を積分する
// 引数         totalPulse: スタート基準の累積距離[pulse], angleDeg: 積算角度[deg]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcXYcie(int32_t totalPulse, float angleDeg)
{
	// ログ間の距離差と区間両端の平均姿勢を使い、疎な瞬時角速度の再積分を避ける。
	float distanceMm = (float)((int64_t)totalPulse - xyPreviousTotalPulse) / PULSE_MILLIMETER;
	float headingRad = (xydegz + angleDeg) * 0.5F * DEG2RAD;
	xycie.x += distanceMm * sinf(headingRad);
	xycie.y += distanceMm * cosf(headingRad);
	xyPreviousTotalPulse = totalPulse;
	xydegz = angleDeg;
}
// モジュール名 clearXYcie
// 処理概要     ログ用XY座標と積算角度を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void clearXYcie(void)
{
	xycie.x = 0;
	xycie.y = 0;
	xydegz = 0;
	xyPreviousTotalPulse = 0;
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 clampMarkerIndex
// 処理概要     マーカーインデックスを有効範囲へ制限する
// 引数         idx: 制限前のインデックス
// 戻り値       制限後のインデックス
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
// ローカル関数 isStraightBeforeMarker
// 処理概要     直前の直線走行距離が評価窓の所定割合以上か判定する
// 引数         encNow: 現在距離[pulse]（未使用）, window_mm: 評価窓[mm], ratio_threshold: 直線率閾値
// 戻り値       true: 直線率が閾値以上, false: 閾値未満
/////////////////////////////////////////////////////////////////////
static bool isStraightBeforeMarker(int32_t encNow, int16_t window_mm, float ratio_threshold)
{
	(void)encNow;
	int32_t straightDistance = straightMeter;	// 直前に直線と判定した走行距離[mm]
	int32_t ratioScaled = (int32_t)(ratio_threshold * 1000.0f); // 直線率閾値を1000倍の整数へ変換する
	int32_t lhs = straightDistance * 1000;	// 直線走行距離を1000倍して比較する
	int32_t rhs = (int32_t)window_mm * ratioScaled;	// 評価窓幅に1000倍の直線率閾値を掛ける
	return lhs >= rhs;	// 直線率が閾値以上か判定する
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 calcDynamicThresholdPulse
// 処理概要     速度と角速度から距離補正の許容誤差を計算する
// 引数         なし
// 戻り値       許容距離誤差[pulse]
/////////////////////////////////////////////////////////////////////
static int32_t calcDynamicThresholdPulse(void)
{
	float speed_mm = encPulse(targetSpeed) * 1000;	// 目標速度[pulse/ms]をmm/sへ変換する
	float mm = 100.0f + (CORR_DYN_COEFF_SPEED * speed_mm) + (CORR_DYN_COEFF_ANG * fabsf(imuVal.gyro.z));	// 基本値100mmに速度と角速度に応じた補正量を加算する
	if (mm < (float)CORR_THRESH_MIN_MM)
	{
		mm = (float)CORR_THRESH_MIN_MM;	// 許容誤差の下限へ制限する
	}
	if (mm > (float)CORR_THRESH_MAX_MM)
	{
		mm = (float)CORR_THRESH_MAX_MM;	// 許容誤差の上限へ制限する
	}
	int16_t mmInt = (int16_t)(mm + 0.5f);	// 四捨五入して整数の距離[mm]にする
	return encMM(mmInt);	// 距離[mm]をパルス数へ換算して返す
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 findNearestMarkerIndex
// 処理概要     近傍マーカーから現在距離に最も近いインデックスを探す
// 引数         encNow: 現在距離[pulse], isCross: クロスライン検出時はtrue
// 戻り値       最寄りマーカーのインデックス、マーカー情報がなければ0
/////////////////////////////////////////////////////////////////////
static int16_t findNearestMarkerIndex(int32_t encNow, bool isCross)
{
	if (numPPAMarry <= 0)
	{
		return 0;	// マーカー情報がなければ0を返す
	}
	int16_t hint = clampMarkerIndex(pathedMarker);	// 推定走行位置を探索のヒントにする
	int16_t center = clampMarkerIndex(lastCorrectedMarker);	// 直前に補正したマーカーを探索の中心にする
	int16_t searchBack = isCross ? MARKER_SEARCH_CROSS_BACK : MARKER_SEARCH_BACK;
	int16_t searchForward = isCross ? MARKER_SEARCH_CROSS_FORWARD : MARKER_SEARCH_FORWARD;
	int16_t lower = center - searchBack;	// 後方探索の開始位置
	int16_t upper = center + searchForward;	// 前方探索の終了位置
	if (hint < lower)
	{
		lower = hint;	// ヒントが手前なら後方探索範囲を広げる
	}
	if (hint > upper)
	{
		upper = hint;	// ヒントが先なら前方探索範囲を広げる
	}
	lower = clampMarkerIndex(lower);
	upper = clampMarkerIndex(upper);
	if (upper < lower)
	{
		int16_t tmp = upper;
		upper = lower;
		lower = tmp;	// 探索範囲の上下限が逆なら入れ替える
	}
	int16_t bestIdx = lower;	// 暫定候補を探索範囲の下限に設定する
	int32_t bestDiff = encNow - markerPos[lower].distance;
	bestDiff = (bestDiff < 0) ? -bestDiff : bestDiff;
	for (int16_t idx = lower + 1; idx <= upper; idx++)
	{
		int32_t diff = encNow - markerPos[idx].distance;
		diff = (diff < 0) ? -diff : diff;
		if (diff < bestDiff)
		{
			bestDiff = diff;
			bestIdx = idx;	// 現在距離により近いマーカーを採用する
		}
	}
	return bestIdx;
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 activateFailSafe
// 処理概要     距離補正失敗時にフェイルセーフの速度制限を適用する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void activateFailSafe(void)
{
	if (failSafeActive)
	{
		return;	// 既に動作中なら再適用しない
	}
	float currentSpeed = (float)targetSpeed / PULSE_MILLIMETER;	// 現在の目標速度[m/s]
	float limitedSpeed = currentSpeed * FAILSAFE_SPEED_SCALE;	// 指定倍率で目標速度を下げる
	setTargetSpeed(limitedSpeed);	// 速度指令を更新する
	boostSpeed = limitedSpeed;	// 参照速度も同期する
	failSafeActive = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 processMarkerEvent
// 処理概要     マーカー検出時の距離補正と速度更新を行う
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void processMarkerEvent(void) {
	// マーカーまたはクロスラインを新たに検出したときの処理
	if (courseMarker > 0 && beforeCourseMarker == 0) {
		cntMarker++; // マーカー検出回数を加算する
		if (optimalTrace == BOOST_DISTANCE) {
			if (numPPAMarry > 0) {
				bool isCross = (courseMarker == CROSSLINE);	// クロスラインなら距離誤差の閾値判定を省く
				bool straightLike = isStraightBeforeMarker(encTotalOptimal, STRAIGHT_WINDOW_MM, STRAIGHT_RATIO_THRESHOLD);	// 直前区間の直線率を判定する
				bool straightPending = straightMarkerPending;	// first marker after straight detection
				bool usedStraightPending = straightPending;
				if (straightLike || isCross || straightPending) {
					if (straightPending) {
						straightMarkerPending = false;
					}
					int16_t nearestIdx = findNearestMarkerIndex(encTotalOptimal, isCross);	// 近傍から現在距離に最も近いマーカーを取得する
					int32_t rawDiff = encTotalOptimal - markerPos[nearestIdx].distance;	// 現在距離とマーカー位置の差[pulse]
					int32_t absDiff = (rawDiff < 0) ? -rawDiff : rawDiff;
					int32_t allowDiff = calcDynamicThresholdPulse();	// 速度と角速度から計算した許容誤差[pulse]
					bool canCorrect = isCross || (absDiff <= allowDiff);	// クロスラインは補正を許可し、それ以外は距離誤差を判定する
					pathedMarker = clampMarkerIndex(nearestIdx);	// 次の探索で使うヒント位置を更新する
					if (canCorrect) {
						if (usedStraightPending) {
							straightMarkerPendingLog = 1;
						}
						int32_t stepLimit = encMM(CORR_STEP_MAX_MM);	// 1回の距離補正量の上限[pulse]
						int32_t diff = rawDiff;
						if (diff > stepLimit) {
							diff = stepLimit;	// 正方向の補正量を上限で制限する
						}
						if (diff < -stepLimit) {
							diff = -stepLimit;
						}
						int32_t errorDistance = encTotalOptimal - DistanceOptimal;	// 補正前の現在距離と目標距離の誤差を保持する
						Control_ApplyMarkerCorrection_p(diff);	// マーカー補正をスリップ補正後の距離パルスへ反映する
						DistanceOptimal = encTotalOptimal - errorDistance;	// 距離誤差を維持したまま目標距離を更新する
						int32_t markerIndex = markerPos[nearestIdx].indexPPAD;	// マーカーに対応するPPADインデックス
						int32_t currentIndex = (int32_t)optimalIndex;
						int32_t nearDev = isCross ? MARKER_INDEX_DEV_CROSS : MARKER_INDEX_DEV_NORMAL;
						// マーカーインデックスを現在のoptimalIndex近傍へ制限する
						if (markerIndex > currentIndex + nearDev)
						{
							markerIndex = currentIndex + nearDev;
						}
						if (markerIndex < currentIndex - nearDev)
						{
							markerIndex = currentIndex - nearDev;
						}
						// クロスライン補正時のインデックス移動量をさらに制限する
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
						boostSpeed = PPAD[optimalIndex].boostSpeed;	// 補正後の区間の目標速度を取得する
						setTargetSpeed(boostSpeed);	// 速度指令へ直ちに反映する
						resetSpeedPID();	// 速度PIDの内部状態をリセットする
						int16_t newPathed = nearestIdx - 2;	// 次回の探索ヒントを少し手前に戻す
						pathedMarker = clampMarkerIndex(newPathed);
						lastCorrectedMarker = nearestIdx;	// 直前の補正マーカー位置を記録する
						missedCorrections = 0;	// 連続補正失敗回数をリセットする
						failSafeActive = false;	// フェイルセーフを解除する
					} else {
						missedCorrections++;	// 補正失敗回数を加算する
						if (missedCorrections >= FAILSAFE_MISS_MAX) {
							activateFailSafe();
						}
					}
				} else {
					missedCorrections++;	// 直線条件を満たさない場合も補正失敗として数える
					if (missedCorrections >= FAILSAFE_MISS_MAX) {
						activateFailSafe();
					}
				}
			}
		} else if(optimalTrace == BOOST_SHORTCUT) {
			// ショートカット走行ではここで距離補正を行わない
		}
	}
	beforeCourseMarker = courseMarker; // 前回のマーカー状態を更新する
}
/////////////////////////////////////////////////////////////////////
// モジュール名 clearMarkerProcessState
// 処理概要     マーカー検出処理の状態を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void clearMarkerProcessState(void) {
	beforeCourseMarker = 0;
	cntMarker = 0;
	straightMeter = 0;	// 直線判定用の走行距離を初期化する
	straightState = false;
	straightMarkerPending = false;
	straightMarkerPendingLog = 0;
	pathedMarker = 0;
	lastCorrectedMarker = 0;
	missedCorrections = 0;
	failSafeActive = false;
}

