//====================================//
// インクルード
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

static bool sd_remount_for_analysis(void)
{
	return (sd_remount() == FR_OK);
}
//====================================//
// グローバル変数の宣
//====================================//
uint8_t optimalTrace = 0;
uint16_t optimalIndex;
int16_t numPPADarry; // path palanning analysis distance (PPAD)
int16_t numPPAMarry; // path palanning analysis marker (PPAM1)
int16_t indexSC;
int16_t pathedMarker = 0;
float boostSpeed;
int32_t DistanceOptimal = 0; // 2次走行用走行距離変数
int16_t analizedNumber = 0;	 // 前回解析したログ番号
int32_t encTotalOptimal = 0; // 2次走行用の距離変数(距離補正をする)
int32_t encPID = 0;			 // 距離制御用の距離変数
float xydegz = 0;
int32_t straightMeter;
bool straightState;
bool straightMarkerPending;
uint8_t straightMarkerPendingLog;

static uint8_t missedCorrections = 0;	// 連続補正失敗回数
static bool failSafeActive = false;	// フェイルセーフ動作中フラグ
static int16_t lastCorrectedMarker = 0;	// 直近で補正したマーカーインデックス

static void logReadSlipIoError(int logNumber, UINT lineNo, FIL *fil, const char *tag);
static float calcDecelLeadMmByRoc(int16_t rocPrev, int16_t rocNow);
static void applyDecelLeadToPpad(int16_t count);
static void applyDecelLeadToArray(float *speed, int16_t count);

AnalysisData PPAD[OPT_BUFF_SIZE];
EventPos markerPos[OPT_BUFF_SIZE];
Courseplot xycie;							   // xy座標値(走行中計算、ログ保存用)
Courseplot shortCutxycie[OPT_SHORT_BUFF_SIZE]; // xy座標値(目標値、ログ保存用)

/////////////////////////////////////////////////////////////////////
// モジュール名 calcROC
// 処理概要     曲率半径の計算
// 引数         velo: エンコーダカウント angvelo: 角速度[rad/s]
// 戻り値       曲率半径[mm]
/////////////////////////////////////////////////////////////////////
float calcROC(int16_t velo, float angvelo, float dt)
{
	// 移動距離 [pulse] → [mm]
    float dl = calcDlMm(velo, dt);  // [mm]
    // 角度変化量 [rad] = ω[deg/s] → rad/s × dt[s]
    float drad = angvelo * DEG2RAD * dt;

    // 絶対値を符号で取る fabs() より高速
    float absDrad = (drad < 0.0f) ? -drad : drad;
    float absDl   = (dl   < 0.0f) ? -dl   : dl;

    // 直線判定：|dl/drad| > ROC_STRAIGHT_TH ⇔ ROC_STRAIGHT_TH * |drad| < |dl|
    // → 除算せずに比較できる
    if (absDrad < 1e-6f || ROC_STRAIGHT_TH * absDrad < absDl) {
        return ROC_STRAIGHT_MAX; // 直線とみなす
    }

    // カーブの場合のみ除算実行
    float R = absDl / absDrad;
    // float absR = (R < 0.0f) ? -R : R;
    // return (absR > ROC_STRAIGHT_TH) ? 2000.0f : R;

	return R;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 saveLogNumber
// 処理概要     解析したログファイルの番号をファイルに保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void saveLogNumber(int16_t fileNumber)
{
	FRESULT fresult;
	FIL fil;
	char fileName[20] = PATH_SETTING;

	strcat(fileName, FILENAME_ANALIZENUMBER);					 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // create file
	if (fresult == FR_OK)
	{
		f_printf(&fil, "%05d", fileNumber);
	}
	f_close(&fil);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getLogNumber
// 処理概要     解析したログファイルの番号を取得する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getLogNumber(void)
{
	FRESULT fresult;
	FIL fil;
	TCHAR log[20];
	char fileName[20] = PATH_SETTING;

	strcat(fileName, FILENAME_ANALIZENUMBER);					// ファイル名追加
	strcat(fileName, ".txt");									// 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_READ); // csvファイルを開く
	if (fresult == FR_OK)
	{
		// 解析済みのログ番号を取得
		f_gets(log, (int)(sizeof(log) / sizeof(log[0])), &fil);
		sscanf(log, "%05d", &analizedNumber);	
		f_close(&fil);
	}

	for (int16_t i = 0; i <= endFileIndex; i++)
	{
		// 解析済みのログ番号に一致するインデックスを保存
		if (analizedNumber == fileNumbers[i])
		{
			fileIndexLog = i;
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 sortInt16Ascending
// 処理概要     最大5要素のint16_t配列を挿入ソートすることでqsort呼び出しを削減
// 引数         values: ソート対象の配列, length: 要素数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void sortInt16Ascending(int16_t *values, uint16_t length)
{
	if (length <= 1)
	{
		return; // 要素数1以下は並べ替え不要
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
// 処理概要     曲率半径の変化量から先行減速距離[mm]を算出する
// 引数         rocPrev:直前の曲率半径[mm], rocNow:現在の曲率半径[mm]
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
		return 0.0f; // 曲率が緩くなる方向は先行減速しない
	}
	if (absPrev <= 0)
	{
		absPrev = 1;
	}

	float changeRatio = (float)(absPrev - absNow) / (float)absPrev; // 変化率(0.0〜1.0)
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
// 引数         count:PPAD配列の有効要素数
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

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // mmを配列ステップ数へ換算
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
			// 減速後速度がすでに決まる位置を手前側へ拡張
			if (PPAD[j].boostSpeed > nowSpeed)
			{
				PPAD[j].boostSpeed = nowSpeed;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 applyDecelLeadToArray
// 処理概要     任意の速度配列の減速区間に先行減速を適用する
// 引数         speed:速度配列, count:有効要素数
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

		int16_t leadStep = (int16_t)ceilf(leadMm / (float)CALCDISTANCE); // mmを配列ステップ数へ換算
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
			// 減速後速度がすでに決まる位置を手前側へ拡張
			if (speed[j] > nowSpeed)
			{
				speed[j] = nowSpeed;
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readLogDistance
// 処理概要     距離基準2次走行の解析
// 引数         ログ番号(ファイル名)
// 戻り値       最適速度配列の最大要素数
//              -1: 解析用配列のサイズを超過
//              -4: ログファイルのオープン失敗
/////////////////////////////////////////////////////////////////////
int16_t readLogDistance(int logNumber)
{
	// ファイル読み込み
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool fileOpened = false; // f_close
	bool retried = false;
	bool errorDetected = false; // 解析途中のエラー発生を検知するフラグ
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9;
	}
	// 解析中はログ書き込みを抑制する
	sd_set_analysis_active(true); // SD/FatFs使用中
	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	retry_open:
	// 解析前に再マウントしてFATの整合を取り直す
	if (!sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup_read;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く

	if (fresult == FR_OK)
	{
		fileOpened = true; // 正常に開けた場合のみクローズ処理を有効化
		// ログデータの取得
		TCHAR log[512];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t time, marker, velo, distance, roc, i = 0;
		float angVelo;
		int32_t numD = 0, numM = 0, cntCurR = 0, numStraight = 0;
		static int16_t ROCbuff[600] = {0};
		int16_t sortROC[CALCDISTANCE / 10];	// sortROCの最大要素数はCALCDISTANCE/10(=5)。動的確保とデバッグprintfを排除するため自動配列を利用
		int32_t straightMeter = 0;
		bool straightState = false;

		// 前処理
		// 構造体配列の初期化
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		TCHAR *header = f_gets(log, log_len, &fil_Read); // 1行目はヘッダなので読み飛ばす
		if (!header && f_error(&fil_Read))
		{
			ret = -5;
			errorDetected = true;
			logReadSlipIoError(logNumber, 0, &fil_Read, "io_fail");
		}

		UINT lineNo = 0;
		// ログデータ取得開始
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
			// 解析処理
			// marker==3: 交差線マーカー
			// marker==2: 左マーカー。直線走行中のみカーブマーカーとして扱う
			if (marker == 3 || (marker == 2 && straightState))
			{
				// カーブマーカーを通過したときにマーカー位置を記録
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;

				if (marker == 2 && straightState)
				{
				// 直線後の左マーカー検出でフラグと距離をリセット
					straightState = false;
					straightMeter = 0;
				}

				numM++; // マーカー解析インデックス更新
			}

			// 一定距離ごとに処理
			if (i > 0 && i % (CALCDISTANCE / 10) == 0) // i==0では処理しない
			{
				int32_t copyCount = cntCurR;	    // 今回ソートする要素数を退避
				if (copyCount > (CALCDISTANCE / 10))
				{
					copyCount = (CALCDISTANCE / 10); // 理論上到達しないが、安全のため上限を適用
				}
				for (int32_t sortIndex = 0; sortIndex < copyCount; sortIndex++)
				{
					sortROC[sortIndex] = ROCbuff[sortIndex]; // 必要な要素のみを手動コピーして中央値算出用に退避
				}
				sortInt16Ascending(sortROC, (uint16_t)copyCount); // 小配列は単純ソートで十分なためqsort呼び出しを削減

				// 曲率半径を記録する
				if (copyCount % 2 == 0)
				{
					// 中央値を記録(配列要素数が偶数のとき) 中央2つの平均値
					PPAD[numD].ROC = (sortROC[copyCount / 2] + sortROC[copyCount / 2 - 1]) / 2;
				}
				else
				{
					// 中央値を記録(配列要素数が奇数のとき)
					PPAD[numD].ROC = sortROC[copyCount / 2];
				}

				PPAD[numD].boostSpeed = asignVelocity(PPAD[numD].ROC); // 曲率半径ごとの速度を計算する

				// 前回の曲率半径と比較(numDが1以上の場合のみ)
				if (numD >= 1 && PPAD[numD].ROC == PPAD[numD - 1].ROC)
				{
					numStraight++;
				}
				else
				{
					numStraight = 0;
				}

				cntCurR = 0; // 曲率半径用配列のカウントクリア
				numD++;		 // 距離解析インデックス更新
				if (numD >= OPT_BUFF_SIZE)
				{
					ret = -1; // 解析用配列のサイズ超過を検出したらエラー扱いとする
					errorDetected = true; // エラーフラグを立てて共通クリーンアップへ遷移
					break; // 即時returnせずループを抜ける
				}
			}
			// 曲率半径の計算
			ROCbuff[cntCurR] = roc;

			if (abs(ROCbuff[cntCurR]) >= 700)
			{
				straightMeter += CALCDISTANCE_SHORTCUT;
			}
			else
			{
				straightMeter = 0;
			}

			// 直線区間が100mm以上続いたら直線走行中と判定し、
			// 次に検出する左マーカーをカーブ開始とするためのフラグを立てる
			if (straightMeter >= 100)
			{
				straightState = true;
			}

			cntCurR++; // 曲率半径用配列のカウント
			i++;
		}

		if (!errorDetected)
		{
			// インデックスが1多くなるので調整
			if (numM > 0)
			{
				numM--;        // 0件時はマーカー数を負にしない
			}
			int32_t numDCount = 0;
			if (numD > 0)
			{
				numD--;        // 0件時は距離要素数を負にしない
				numDCount = numD + 1;        // 要素数に戻して加減速調整で使用
			}
			else
			{
				numDCount = numD;        // 要素数0の場合はそのまま利用
			}
			numD = numDCount;
			applyDecelLeadToPpad((int16_t)numD); // 曲率変化に応じて、減速到達位置を手前へ寄せる

			// 目標速度配列の整形 加減速が間に合うように距離を調整する
			float acceleration, elapsedTime, dv, dl;

			// 最初の要素は調整しない
			dl = (float)CALCDISTANCE / 1000;

			// numDを件数として扱うため、以下のループでも境界外アクセスは発生しない

			// 加速インデックス1から末尾まで平滑化
			for (int32_t idx = 1; idx < numD; idx++)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx - 1].boostSpeed);	// 区間速度差
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 速度差が極小なら補正不要
				}
				elapsedTime = fabs(dl / dv);		// 区間時間
				acceleration = dv / elapsedTime;	// 実測加速度
				if (acceleration > MACHINEACCELE)
				{
					PPAD[idx].boostSpeed = PPAD[idx - 1].boostSpeed + (MACHINEACCELE * dl);
				}
			}

			// 減速インデックス末尾から先頭まで平滑化
			for (int32_t idx = numD - 2; idx >= 0; idx--)
			{
				dv = (PPAD[idx].boostSpeed - PPAD[idx + 1].boostSpeed);	// 区間速度差
				if (fabsf(dv) < 1e-6f)
				{
					continue;	// 速度差が極小なら補正不要
				}
				elapsedTime = fabs(dl / dv);
				acceleration = dv / elapsedTime;
				if (acceleration > MACHINEDECREACE)
				{
					PPAD[idx].boostSpeed = PPAD[idx + 1].boostSpeed + (MACHINEDECREACE * dl);
				}
			}

#ifdef WRITE_BOOSTSPEED_LOG
			// 平滑化後の目標速度配列をSDカードへ記録する
			FIL fil_Boost;
			FRESULT fresult_Boost;
			char boostFileName[32];
			snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
			fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
			if (fresult_Boost == FR_OK)
			{
				// CSVヘッダを書き込み、平滑化済みのboostSpeedを順番に保存する
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
			// エラー発生時は整形処理を行わず解析結果を破棄
		}
	}
	else
	{
		ret = -4;
		errorDetected = true; // ファイルオープン失敗時もエラー状態として扱う
	}

	if (ret == -5 && !retried)
	{
		// I/Oエラー時は一度だけ再マウント＆再オープンを試す
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
		f_close(&fil_Read); // オープン成功時のみクローズを実施
	}
	if (lock_acquired)
	{
		// 解析終了（書き込み抑制解除）
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	// printf("Analysis distance end\n");

	if (ret >= 0)
	{
		// 正常終了時のみ解析済み情報を更新
		saveLogNumber(logNumber);
		analizedNumber = logNumber;

		// 2次走行フラグ 距離基準2次走行
		optimalTrace = BOOST_DISTANCE;
	}
	else
	{
		// エラー発生時は状態更新を行わず呼び出し元に返却
	}

	return ret;
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 parseSecondLogLine
// 処理概要	 2次走行ログの必要列のみを抽出する
// 引数	 line: 1行文字列, 各出力先ポインタ
// 戻り値	 解析成功ならtrue
/////////////////////////////////////////////////////////////////////
static bool parseSecondLogLine(const char *line, uint8_t *courseMarker, int32_t *encTotal,
		int16_t *roc, float *targetSpeedLog, int16_t *optimalIdx, uint8_t *slipLong, uint8_t *slipLat)
{
	// 追加: ヘッダ/空行判定のため先頭の有効文字を確認する
	const char *p = line;
	while (*p == ' ' || *p == '\t')
	{
		p++;
	}
	if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '+'))
	{
		return false;	// 数値で始まらない行はスキップ
	}

	// 追加: カンマを数えながら必要列だけを抽出する
	int col = 0;
	const char *field = p;
	bool gotOptimal = false;
	while (1)
	{
		// 追加: 区切り文字(カンマ/改行/終端)でフィールドを確定する
		if (*p == ',' || *p == '\n' || *p == '\r' || *p == '\0')
		{
			char *endptr = NULL;
			switch (col)
			{
			case 3:
				*courseMarker = (uint8_t)strtol(field, &endptr, 10);
				break;
			case 4:
				*encTotal = (int32_t)strtol(field, &endptr, 10);
				break;
			case 5:
				*roc = (int16_t)strtol(field, &endptr, 10);
				break;
			case 6:
				*targetSpeedLog = strtof(field, &endptr);
				break;
			case 7:
				*optimalIdx = (int16_t)strtol(field, &endptr, 10);
				gotOptimal = true;
				break;
			case 8:
				*slipLong = (uint8_t)strtol(field, &endptr, 10);
				break;
			case 9:
				*slipLat = (uint8_t)strtol(field, &endptr, 10);
				break;
			default:
				break;
			}
			if (*p == ',')
			{
				col++;
				if (col > 9)
				{
					break;	// 追加: 必要列を超えたら早期終了
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
		// ファイル終端へ移動(ファイル追記の準備)
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
// モジュール名 readLogDistanceSlip
// 処理概要     2次走行ログからスリップを考慮した3次走行用速度計画を作成する
// 引数         ログ番号(ファイル名)
// 戻り値       最適速度配列の最大要素数
/////////////////////////////////////////////////////////////////////
int16_t readLogDistanceSlip(int logNumber)
{
	int16_t baseLogNumber = analizedNumber;
	if (baseLogNumber <= 0)
	{
		baseLogNumber = logNumber; // 追加: 1走目ログが無ければ直前ログを使用
	}

	// 追加: 1走目ログをreadLogDistance相当で解析して速度計画とマーカー配列を作成
	int16_t baseRet = readLogDistance(baseLogNumber);
	if (baseRet < 0)
	{
		return baseRet;
	}
	int16_t baseCount = numPPADarry;
	if (baseCount <= 0)
	{
		return -2; // 解析対象が無い
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
		return -9; // SD/FatFs使用中
	}
	// 解析中はログ書き込みを抑制する
	sd_set_analysis_active(true);

	// 追加: 解析用バッファ・配列は静的領域で確保してスタックを節約
	static uint16_t sampleCnt[OPT_BUFF_SIZE];
	static float v2Max[OPT_BUFF_SIZE];
	static float rocAbsSum[OPT_BUFF_SIZE];
	static uint16_t rocCnt[OPT_BUFF_SIZE];
	static uint16_t slipLongCnt[OPT_BUFF_SIZE];
	static uint16_t slipLatCnt[OPT_BUFF_SIZE];
	static float risk[OPT_BUFF_SIZE];
	static float riskExpanded[OPT_BUFF_SIZE];
	static float v3[OPT_BUFF_SIZE];

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	retry_open_slip:
	// 解析前に再マウントしてFATの整合を取り直す
	if (!sd_remount_for_analysis()) {
		ret = -6;
		goto cleanup;
	}
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く
	if (fresult != FR_OK)
	{
		ret = -4; // ログファイルのオープン失敗
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

	// 1行目はヘッダなので読み飛ばす
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
			ret = -2; // 解析対象が無い
		}
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

		if (!parseSecondLogLine((const char *)log, &courseMarker, &encTotal, &roc,
				&targetSpeedLog, &optimalIdx, &slipLong, &slipLat))
		{
			continue;	// 追加: ヘッダ/空行は解析しない
		}
		if (optimalIdx < 0 || optimalIdx >= baseCount)
		{
			ret = -1;	// 追加: 解析用配列の上限超過
			break;
		}

		// 追加: 集計(サンプル数/最大速度/ROC平均/スリップ回数)
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

		// 追加: 2次ログからマーカー位置を再構築する
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
		// I/Oエラー時は一度だけ再マウント＆再オープンを試す
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
		// 解析終了（書き込み抑制解除）
		sd_set_analysis_active(false);
		sd_fatfs_unlock();
	}

	if (ret < 0)
	{
		return ret;
	}
	if (maxOptimalIndex < 0)
	{
		return -2;	// 追加: 解析対象が無い
	}

	// 追加: 欠番optimalIndexは前値で埋める(前方埋め)
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		if (sampleCnt[i] == 0 && i > 0)
		{
			rocAbsSum[i] = rocAbsSum[i - 1];	// 追加: ROC平均用の前方埋め
			rocCnt[i] = rocCnt[i - 1];			// 追加: ROC平均用の前方埋め
		}
	}

	// 追加: risk(0..1)を作る
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

	// 追加: 近傍へリスク拡張
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

	// 追加: v3を更新(減速/増速)
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

	// 追加: 2次の速度変化量以内に収める(前後パス)
	applyDecelLeadToArray(v3, (int16_t)(maxOptimalIndex + 1)); // 曲率変化に応じて、減速到達位置を手前へ寄せる
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

	// 追加: PPADへ反映
	for (int16_t i = 0; i <= maxOptimalIndex; i++)
	{
		PPAD[i].boostSpeed = v3[i];
	}

	ret = numPPADarry;

#ifdef WRITE_BOOSTSPEED_LOG
	// 平滑化後の目標速度配列をSDカードへ記録する
	FIL fil_Boost;
	FRESULT fresult_Boost;
	char boostFileName[32];
	snprintf(boostFileName, sizeof(boostFileName), "%sboost_%05d.csv", PATH_SETTING, logNumber);
	fresult_Boost = f_open(&fil_Boost, boostFileName, FA_CREATE_ALWAYS | FA_WRITE);
	if (fresult_Boost == FR_OK)
	{
		// CSVヘッダを書き込み、平滑化済みのboostSpeedを順番に保存する
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

	// 追加: 解析済み情報を更新
	optimalTrace = BOOST_DISTANCE;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 asignVelocity
// 処理概要     曲率半径ごとの最適速度を割り当てる
// 引数
// 戻り値       なし
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
// 処理概要     float型の比較
// 引数
// 戻り値       なし
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
// モジュール名 readLogDistance
// 処理概要     距離基準2次走行の解析
// 引数         ログ番号(ファイル名)
// 戻り値       最適速度配列の最大要素数
/////////////////////////////////////////////////////////////////////
int16_t readLogTest(int logNumber)
{
	// ファイル読み込み
	FIL fil_Read;
	FRESULT fresult;
	char fileName[10];
	int16_t ret = 0;
	bool lock_acquired = sd_fatfs_lock(200);

	if (!lock_acquired)
	{
		return -9; // SD/FatFs使用中
	}

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く

	if (fresult == FR_OK)
	{
		TCHAR log[512];
		const int log_len = (int)(sizeof(log) / sizeof(log[0]));
		int32_t time, marker, velo, distance;
		float angVelo;
		int32_t startEnc = 0, numD = 0, numM = 0, beforeMarker = 0;
		bool analysis = false;

		// 前処理
		// 構造体配列の初期化
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		// ログデータ取得開始
		while (f_gets(log, log_len, &fil_Read))
		{
			sscanf(log, "%d,%d,%f,%d,%d", &time, &velo, &angVelo, &marker, &distance);

			// 解析処理
			if (marker == 1 && beforeMarker == 0)
			{
				// ゴールマーカーを通過したときにフラグ反転
				analysis = !analysis;
				startEnc = distance;
			}
			else if (marker == 0 && beforeMarker == 2)
			{
				// カーブマーカーを通過したときにマーカー位置を記録
				markerPos[numM].distance = distance;
				markerPos[numM].indexPPAD = numD;
				numM++; // マーカー解析インデックス更新
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

	// 解析済みのログ番号を保存
	// saveLogNumber(logNumber);
	analizedNumber = logNumber;

	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcXYcies (cie=Coordinate)
// 処理概要     ログから走行軌跡のXY座標を計算する
// 引数         ログ番号(ファイル名)
// 戻り値       解析した配列の要素数
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
		return -9; // SD/FatFs使用中
	}

	// ファイル読み込み
	snprintf(fileName, sizeof(fileName), "%d", logNumber);				// 数値を文字列に変換
	strcat(fileName, ".csv");											// 拡張子を追加
	fresult1 = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // ログファイルを開く
	if (fresult1 != FR_OK)
	{
		// ログファイルのオープンに失敗した場合はエラーを返す
		ret = -5;       // ログファイルオープンエラー
		if (lock_acquired)
		{
			sd_fatfs_unlock();
		}
		return ret;
	}
	fresult2 = f_open(&fil_Plot, "./plot/plot.csv", FA_CREATE_ALWAYS | FA_WRITE); // csvファイルを開く
	if (fresult2 != FR_OK)
	{
		// プロットファイルのオープンに失敗した場合はエラーを返す
		f_close(&fil_Read);
		ret = -6;       // プロットファイルオープンエラー
		if (lock_acquired)
		{
			sd_fatfs_unlock();
		}
		return ret;
	}

	// プロットファイルが開けたので解析を開始
	// ログデータの取得
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

	// 配列の初期化
	memset(&xValues, 0, sizeof(float) * SHORTCUTWINDOW);
	memset(&yValues, 0, sizeof(float) * SHORTCUTWINDOW);
	memset(&degzValues, 0, sizeof(float) * SHORTCUTWINDOW);
	indexSC = 0;

	// ショートカット軌跡初期値の設定
	shortCutxycie[indexSC].x = 0;
	shortCutxycie[indexSC].y = 0;
	shortCutxycie[indexSC].w = 0;
	indexSC++;

	// plotファイルのヘッダ書き込み
	f_printf(&fil_Plot, "xm,ym,degzm\n");
	
	f_gets(log, log_len, &fil_Read); // 1行目はヘッダなので読み飛ばす

	// ログデータ取得開始
	while (f_gets(log, log_len, &fil_Read) != NULL)
	{
		sscanf(log, "%d,%d,%f,%d,%d", &time, &velo, &angVelo, &marker, &distance);

		dt = (float)(time - beforeTime) / 1000;		// 時間[s]

		degz = degz + (angVelo * dt);			   	// 角度
		degzR = degz * DEG2RAD;					   	// [rad]に変換
		velocity = (float)velo / PALSE_MILLIMETER;	// 速度
		distEnc += velo * (time - beforeTime);		// 距離計測

		// 座標計算
		x = x + (velocity * sin(degzR));
		y = y + (velocity * cos(degzR));

		// リングバッファに座標を保存
		xValues[i & (SHORTCUTWINDOW - 1)] = x;
		yValues[i & (SHORTCUTWINDOW - 1)] = y;
		degzValues[i & (SHORTCUTWINDOW - 1)] = degz;

		// リングバッファの総和計算前に初期化
		xm = ym = degzm = 0.0f; // 各周回で正しい平均値を得るためリセット
		// リングバッファの総和を計算
		for (j = 0; j < SHORTCUTWINDOW; j++)
		{
			xm += xValues[j];
			ym += yValues[j];
			degzm += degzValues[j];
		}

		// 移動平均を計算(ショートカット座標)
		xm /= SHORTCUTWINDOW;
		ym /= SHORTCUTWINDOW;
		degzm /= SHORTCUTWINDOW;
		if (distEnc - startEnc >= encMM(CALCDISTANCE_SHORTCUT))
		{
			// バッファ上限に達していないか確認
			if (indexSC < OPT_SHORT_BUFF_SIZE)
			{
				shortCutxycie[indexSC].x = xm;
				shortCutxycie[indexSC].y = ym;
				startEnc = distEnc; // 距離計測開始位置を更新
				indexSC++; // バッファの次の位置へ
			}
			else
			{
				// 上限超過: エラー番号-7を設定しループを終了
				ret = -7;
				break;
			}
		}

		i++;
		beforeTime = time;
	}

	// ショートカット座標からyaw軸角度を計算
	float xe = 0, ye = 0;
	float theta = 0, thetaBefore = 90, thetae;

	degz = 0;
	// plotファイルに初期値記録
	f_printf(&fil_Plot, "%d,%d,%d\n", (int32_t)(shortCutxycie[0].x * 10000), (int32_t)(shortCutxycie[0].y * 10000), (int32_t)(shortCutxycie[0].w * 10000));

	for (i = 1; i < indexSC; i++)
	{
		xe = shortCutxycie[i].x - shortCutxycie[i - 1].x; // x座標の移動量
		ye = shortCutxycie[i].y - shortCutxycie[i - 1].y; // y座標の移動量

		theta = atan2(ye, xe) * RAD2DEG; // [deg]に変換

		// 2直線のなす角を計算
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

		shortCutxycie[i].w = degz; // yaw軸角度
		// plotファイルに書き込み
		int snlen = snprintf((char *)plotStr, sizeof(plotStr), "%f,%f,%f\n", shortCutxycie[i].x, shortCutxycie[i].y, shortCutxycie[i].w); // 戻り値で書き込み長を確認
		if (snlen < 0 || snlen >= sizeof(plotStr))
		{
			// snprintfが失敗した場合やバッファが不足した場合はエラー番号-8を設定して処理を中断する
			ret = -8;
			break;
		}
		f_puts((TCHAR *)plotStr, &fil_Plot);
		
		thetaBefore = theta; // 前回のyaw軸角度を更新
	}

	if (ret >= 0)
	{
		// ループ内でエラーがなければ解析した要素数を返す
		ret = indexSC;
	}

	// ファイルクローズ
	f_close(&fil_Read);
	f_close(&fil_Plot);
	if (lock_acquired)
	{
		sd_fatfs_unlock();
	}

	// エラー時にはログ番号保存やフラグ設定をスキップする
	if (ret >= 0)
	{
		// 解析済みのログ番号を保存
		saveLogNumber(logNumber);
		analizedNumber = logNumber;

		// 2次走行フラグ 距離基準2次走行
		optimalTrace = BOOST_SHORTCUT;
	}

	return ret;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcXYcie (cie=Coordinate)
// 処理概要     走行中にxy座標を計算しグローバル変数に保存する
// 引数         encpulse:エンコーダパルス angVelo:角速度[deg/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcXYcie(int16_t encpulse, float angVelo, float dt)
{
	static float velocity, degzR;

	xydegz = xydegz + (angVelo * dt);		// 角度
	degzR = xydegz * (M_PI / 180.0F);		// [rad]に変換
	velocity = (float)encpulse / PALSE_MILLIMETER * 1000; // 速度

	xycie.x = xycie.x + (velocity * sin(degzR) * dt);
	xycie.y = xycie.y + (velocity * cos(degzR) * dt);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 clearXYcie (cie=Coordinate)
// 処理概要     グローバル変数xycieの初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void clearXYcie(void)
{
	xycie.x = 0;
	xycie.y = 0;
	xydegz = 0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setShortCutTarget
// 処理概要     グローバル変数xycieの初期化
// 引数         なし
// 戻り値       なし
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
// ローカル関数 clampMarkerIndex
// 処理概要	 マーカーインデックスを安全な範囲に収める
// 引数	 idx: 候補インデックス
// 戻り値	 範囲内にクランプしたインデックス
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
// 処理概要	 直前区間の直線率を判定する
// 引数	 encNow: 現在エンコーダ値, window_mm: 評価窓[mm], ratio_threshold: 直線率閾値
// 戻り値	 閾値以上ならtrue
/////////////////////////////////////////////////////////////////////
static bool isStraightBeforeMarker(int32_t encNow, int16_t window_mm, float ratio_threshold)
{
	(void)encNow;
	int32_t straightDistance = straightMeter;	// 直近で直線と判定できた距離[mm]
	int32_t ratioScaled = (int32_t)(ratio_threshold * 1000.0f); // 閾値を固定小数点(×1000)へ変換
	int32_t lhs = straightDistance * 1000;	// 評価窓に対する実際の直線率（分子側）
	int32_t rhs = (int32_t)window_mm * ratioScaled;	// 閾値 × 窓幅（分母側を同倍率で換算）
	return lhs >= rhs;	// 直線率が閾値以上かどうか判定
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 calcDynamicThresholdPulse
// 処理概要	 補正許容値を速度・角速度から動的に算出する
// 引数	 なし
// 戻り値	 許容距離[パルス]
/////////////////////////////////////////////////////////////////////
static int32_t calcDynamicThresholdPulse(void)
{
	float speed_mm = encPulse(targetSpeed) * 1000;	// 速度[pulse]をmm/sへ変換
	float mm = 100.0f + (CORR_DYN_COEFF_SPEED * speed_mm) + (CORR_DYN_COEFF_ANG * fabsf(BMI088val.gyro.z));	// 基本値100mmに速度・角速度の補正を加算
	if (mm < (float)CORR_THRESH_MIN_MM)
	{
		mm = (float)CORR_THRESH_MIN_MM;	// 下限を下回らないようクランプ
	}
	if (mm > (float)CORR_THRESH_MAX_MM)
	{
		mm = (float)CORR_THRESH_MAX_MM;	// 上限を超えた場合は上限で固定
	}
	int16_t mmInt = (int16_t)(mm + 0.5f);	// 四捨五入して整数mmへ
	return encMM(mmInt);	// mm→パルスへ換算して返却
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 findNearestMarkerIndex
// 処理概要	 近傍のマーカーから最も近いインデックスを探索する
// 引数	 encNow: 現在エンコーダ値
// 戻り値	 最寄りマーカーのインデックス
/////////////////////////////////////////////////////////////////////
static int16_t findNearestMarkerIndex(int32_t encNow, bool isCross)
{
	if (numPPAMarry <= 0)
	{
		return 0;	// マーカー情報が無い場合は先頭を返す
	}
	int16_t hint = clampMarkerIndex(pathedMarker);	// 推定走行位置からのヒント
	int16_t center = clampMarkerIndex(lastCorrectedMarker);	// 直近で補正したマーカーを中心にする
	int16_t searchBack = isCross ? MARKER_SEARCH_CROSS_BACK : MARKER_SEARCH_BACK;
	int16_t searchForward = isCross ? MARKER_SEARCH_CROSS_FORWARD : MARKER_SEARCH_FORWARD;
	int16_t lower = center - searchBack;	// 後方探索開始位置
	int16_t upper = center + searchForward;	// 前方探索終了位置
	if (hint < lower)
	{
		lower = hint;	// ヒントがより手前なら後方範囲を拡張
	}
	if (hint > upper)
	{
		upper = hint;	// ヒントが先なら前方範囲を拡張
	}
	lower = clampMarkerIndex(lower);
	upper = clampMarkerIndex(upper);
	if (upper < lower)
	{
		int16_t tmp = upper;
		upper = lower;
		lower = tmp;	// 上下が逆転した場合は入れ替え
	}
	int16_t bestIdx = lower;	// 暫定候補を下限に設定
	int32_t bestDiff = encNow - markerPos[lower].distance;
	bestDiff = (bestDiff < 0) ? -bestDiff : bestDiff;
	for (int16_t idx = lower + 1; idx <= upper; idx++)
	{
		int32_t diff = encNow - markerPos[idx].distance;
		diff = (diff < 0) ? -diff : diff;
		if (diff < bestDiff)
		{
			bestDiff = diff;
			bestIdx = idx;	// より近いマーカーを採用
		}
	}
	return bestIdx;
}
/////////////////////////////////////////////////////////////////////
// ローカル関数 activateFailSafe
// 処理概要	 補正失敗時のフェイルセーフ速度制限を適用する
// 引数	 なし
// 戻り値	 なし
/////////////////////////////////////////////////////////////////////
static void activateFailSafe(void)
{
	if (failSafeActive)
	{
		return;	// 既に発動済みなら何もしない
	}
	float currentSpeed = (float)targetSpeed / PALSE_MILLIMETER;	// 現在の目標速度[m/s]
	float limitedSpeed = currentSpeed * FAILSAFE_SPEED_SCALE;	// 指定倍率で安全側に減速
	setTargetSpeed(limitedSpeed);	// 速度指令を更新
	boostSpeed = limitedSpeed;	// 参照速度も同期
	failSafeActive = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 processMarkerEvent
// 処理概要     マーカー通過時の処理をまとめる
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void processMarkerEvent(void) {
	// カーブマーカー,クロスラインを検出した時の処理
	if (courseMarker > 0 && beforeCourseMarker == 0) {
		cntMarker++; // マーカーカウント
		if (optimalTrace == BOOST_DISTANCE) {
			if (numPPAMarry > 0) {
				bool isCross = (courseMarker == CROSSLINE);	// クロスラインなら無条件補正
				bool straightLike = isStraightBeforeMarker(encTotalOptimal, STRAIGHT_WINDOW_MM, STRAIGHT_RATIO_THRESHOLD);	// 直線率判定
				bool straightPending = straightMarkerPending;	// first marker after straight detection
				bool usedStraightPending = straightPending;
				if (straightLike || isCross || straightPending) {
					if (straightPending) {
						straightMarkerPending = false;
					}
					int16_t nearestIdx = findNearestMarkerIndex(encTotalOptimal, isCross);	// 近傍から最適マーカーを取得
					int32_t rawDiff = encTotalOptimal - markerPos[nearestIdx].distance;	// 現在距離との差分[パルス]
					int32_t absDiff = (rawDiff < 0) ? -rawDiff : rawDiff;
					int32_t allowDiff = calcDynamicThresholdPulse();	// 動的に算出した許容誤差
					bool canCorrect = isCross || (absDiff <= allowDiff);	// クロスは即補正、それ以外は閾値判定
					pathedMarker = clampMarkerIndex(nearestIdx);	// ヒント位置を更新
					if (canCorrect) {
						if (usedStraightPending) {
							straightMarkerPendingLog = 1;
						}
						int32_t stepLimit = encMM(CORR_STEP_MAX_MM);	// 段階補正の上限量[パルス]
						int32_t diff = rawDiff;
						if (diff > stepLimit) {
							diff = stepLimit;	// 段階補正で切り詰め
						}
						if (diff < -stepLimit) {
							diff = -stepLimit;
						}
						int32_t errorDistance = encTotalOptimal - DistanceOptimal;	// 補正前の距離誤差を保持
						Control_ApplyMarkerCorrection_p(diff);	// マーカー補正をスリップ補正後パルスへ反映
						DistanceOptimal = encTotalOptimal - errorDistance;	// 誤差を維持したまま目標距離を更新
						int32_t markerIndex = markerPos[nearestIdx].indexPPAD;	// PPAD側の対応インデックス
						int32_t currentIndex = (int32_t)optimalIndex;
						int32_t nearDev = isCross ? MARKER_INDEX_DEV_CROSS : MARKER_INDEX_DEV_NORMAL;
						// marker indexを現在のoptimalIndex近傍に拘束する
						if (markerIndex > currentIndex + nearDev)
						{
							markerIndex = currentIndex + nearDev;
						}
						if (markerIndex < currentIndex - nearDev)
						{
							markerIndex = currentIndex - nearDev;
						}
						// クロスライン補正時は1回でのジャンプ量をさらに制限する
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
						boostSpeed = PPAD[optimalIndex].boostSpeed;	// 区間速度を取得
						setTargetSpeed(boostSpeed);	// 目標速度へ即反映
						resetSpeedPID();	// PID内部状態を同期
						int16_t newPathed = nearestIdx - 2;	// 次回探索は少し手前から
						pathedMarker = clampMarkerIndex(newPathed);
						lastCorrectedMarker = nearestIdx;	// 直近補正位置を記録
						missedCorrections = 0;	// 失敗カウンタをリセット
						failSafeActive = false;	// フェイルセーフ解除
					} else {
						missedCorrections++;	// 補正失敗をカウント
						if (missedCorrections >= FAILSAFE_MISS_MAX) {
							activateFailSafe();
						}
					}
				} else {
					missedCorrections++;	// 直線条件不成立でも失敗扱い
					if (missedCorrections >= FAILSAFE_MISS_MAX) {
						activateFailSafe();
					}
				}
			}
		} else if(optimalTrace == BOOST_SHORTCUT) {
			// ショートカット基準2次走行のとき
		}
	}
	beforeCourseMarker = courseMarker; // 前回のマーカー状態を更新
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cleaerMarkerProcessState
// 処理概要     マーカー通過処理状態の初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void clearMarkerProcessState(void) {
	beforeCourseMarker = 0;
	cntMarker = 0;
	straightMeter = 0;	// 追加: 直線判定距離を初期化
	straightState = false;
	straightMarkerPending = false;
	straightMarkerPendingLog = 0;
	pathedMarker = 0;
	lastCorrectedMarker = 0;
	missedCorrections = 0;
	failSafeActive = false;
}
