//====================================//
// インクルード
//====================================//
#include "courseAnalysis.h"
#include "fatfs.h"
#include "PIDcontrol.h"
#include "markerSensor.h"
#include "BMI088.h"
//====================================//
// グローバル変数の宣
//====================================//
static const float invPulseConst = 10.0F / PALSE_MILLIMETER;	// エンコーダパルスをミリメートル換算する係数
#define STRAIGHT_WINDOW_MM		150	// 直線判定に用いる距離窓[mm]
#define STRAIGHT_RATIO_THRESHOLD	0.80f	// 直線率の閾値
#define CORR_THRESH_MIN_MM		120	// 補正許容誤差の下限[mm]
#define CORR_THRESH_MAX_MM		400	// 補正許容誤差の上限[mm]
#define CORR_STEP_MAX_MM		200	// 1回あたりの最大補正量[mm]
#define FAILSAFE_MISS_MAX		2	// 補正失敗回数の閾値
#define FAILSAFE_SPEED_SCALE	0.85f	// フェイルセーフ時の速度倍率
#define MARKER_SEARCH_BACK		3	// マーカー探索時の後方探索幅
#define MARKER_SEARCH_FORWARD	3	// マーカー探索時の前方探索幅
#define CORR_DYN_COEFF_SPEED	0.02f	// 速度依存補正係数
#define CORR_DYN_COEFF_ANG	0.5f	// 角速度依存補正係数
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
static uint8_t missedCorrections = 0;	// 連続補正失敗回数
static bool failSafeActive = false;	// フェイルセーフ動作中フラグ
static int16_t lastCorrectedMarker = 0;	// 直近で補正したマーカーインデックス

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
    float dl = (float)velo * invPulseConst;  // [mm]
    // 角度変化量 [rad] = ω[deg/s] → rad/s × dt[s]
    float drad = angvelo * DEG2RAD * dt;

    // 絶対値を符号で取る fabs() より高速
    float absDrad = (drad < 0.0f) ? -drad : drad;
    float absDl   = (dl   < 0.0f) ? -dl   : dl;

    // 直線判定：|dl/drad| > STRAIGHTTH ⇔ STRAIGHTTH * |drad| < |dl|
    // → 除算せずに比較できる
    if (absDrad < 1e-6f || STRAIGHTTH * absDrad < absDl) {
        return 2000.0f; // 直線とみなす
    }

    // カーブの場合のみ除算実行
    float R = dl / drad;
    float absR = (R < 0.0f) ? -R : R;
    return (absR > STRAIGHTTH) ? 2000.0f : R;
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
	uint8_t fileName[20] = PATH_SETTING;

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
	uint8_t fileName[20] = PATH_SETTING;

	strcat(fileName, FILENAME_ANALIZENUMBER);					// ファイル名追加
	strcat(fileName, ".txt");									// 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_READ); // csvファイルを開く
	if (fresult == FR_OK)
	{
		// 解析済みのログ番号を取得
		f_gets(log, sizeof(log), &fil);
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
	uint8_t fileName[10];
	int16_t ret = 0;
	uint32_t i;
	bool fileOpened = false; // f_closeの要否を判断するためのフラグ
	bool errorDetected = false; // 解析途中のエラー発生を検知するフラグ

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く

	if (fresult == FR_OK)
	{
		fileOpened = true; // 正常に開けた場合のみクローズ処理を有効化
		// ログデータの取得
		TCHAR log[512];
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

		f_gets(log, sizeof(log), &fil_Read); // 1行目はヘッダなので読み飛ばす
		// ログデータ取得開始
		while (f_gets(log, sizeof(log), &fil_Read))
		{
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

			// for (i = 0; i < numD; i++)
			// {
			// 	printf("%f\n", PPAD[i].boostSpeed);
			// }

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

	if (fileOpened)
	{
		f_close(&fil_Read); // オープン成功時のみクローズを実施
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
	uint8_t fileName[10];
	int16_t ret = 0;
	uint32_t i;

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く

	if (fresult == FR_OK)
	{
		TCHAR log[512];
		int32_t time, marker, velo, distance;
		float angVelo;
		int32_t startEnc = 0, numD = 0, numM = 0, cntCurR = 0, beforeMarker = 0;
		bool analysis = false;
		float ROCbuff[600] = {0};
		float *sortROC;

		// 前処理
		// 構造体配列の初期化
		memset(&PPAD, 0, sizeof(AnalysisData) * OPT_BUFF_SIZE);

		// ログデータ取得開始
		while (f_gets(log, sizeof(log), &fil_Read))
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
	uint8_t fileName[10];
	int16_t ret = 0;

	// ファイル読み込み
	snprintf(fileName, sizeof(fileName), "%d", logNumber);				// 数値を文字列に変換
	strcat(fileName, ".csv");											// 拡張子を追加
	fresult1 = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // ログファイルを開く
	if (fresult1 != FR_OK)
	{
		// ログファイルのオープンに失敗した場合はエラーを返す
		ret = -5;       // ログファイルオープンエラー
		return ret;
	}
	fresult2 = f_open(&fil_Plot, "./plot/plot.csv", FA_CREATE_ALWAYS | FA_WRITE); // csvファイルを開く
	if (fresult2 != FR_OK)
	{
		// プロットファイルのオープンに失敗した場合はエラーを返す
		f_close(&fil_Read);
		ret = -6;       // プロットファイルオープンエラー
		return ret;
	}

	// プロットファイルが開けたので解析を開始
	// ログデータの取得
	TCHAR log[512];
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
	
	f_gets(log, sizeof(log), &fil_Read); // 1行目はヘッダなので読み飛ばす

	// ログデータ取得開始
	while (f_gets(log, sizeof(log), &fil_Read) != NULL)
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
void calcXYcie(float encpulse, float angVelo, float dt)
{
	static float velocity, degzR;

	xydegz = xydegz + (angVelo * dt);		// 角度
	degzR = xydegz * (M_PI / 180.0F);		// [rad]に変換
	velocity = encpulse / PALSE_MILLIMETER; // 速度

	xycie.x = xycie.x + (velocity * sin(degzR));
	xycie.y = xycie.y + (velocity * cos(degzR));
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
	float speed_mm = fabsf((float)targetSpeed) * invPulseConst;	// 速度[pulse]をmm/sへ変換
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
static int16_t findNearestMarkerIndex(int32_t encNow)
{
	if (numPPAMarry <= 0)
	{
		return 0;	// マーカー情報が無い場合は先頭を返す
	}
	int16_t hint = clampMarkerIndex(pathedMarker);	// 推定走行位置からのヒント
	int16_t center = clampMarkerIndex(lastCorrectedMarker);	// 直近で補正したマーカーを中心にする
	int16_t lower = center - MARKER_SEARCH_BACK;	// 後方探索開始位置
	int16_t upper = center + MARKER_SEARCH_FORWARD;	// 前方探索終了位置
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
	static uint8_t beforeModeCurve = 0; // 前回のカーブモード
	bool checkDistance = false;	  // 距離補正状態

	if (modeCurve == 0 && beforeModeCurve > 0) {
		// カーブモードからストレートモードに変化したとき
		checkDistance = false;   // 距離補正状態をクリア
	}
	// カーブマーカー,クロスラインを検出した時の処理
	if (courseMarker > 0 && beforeCourseMarker == 0) {
		cntMarker++; // マーカーカウント
		if (optimalTrace == BOOST_DISTANCE) {
			if (numPPAMarry > 0) {
				bool isCross = (courseMarker == CROSSLINE);	// クロスラインなら無条件補正
				bool straightLike = straightState || isStraightBeforeMarker(encTotalOptimal, STRAIGHT_WINDOW_MM, STRAIGHT_RATIO_THRESHOLD);	// 直線成立または直線率判定
				if (straightLike || isCross) {
					int16_t nearestIdx = findNearestMarkerIndex(encTotalOptimal);	// 近傍から最適マーカーを取得
					int32_t rawDiff = encTotalOptimal - markerPos[nearestIdx].distance;	// 現在距離との差分[パルス]
					int32_t absDiff = (rawDiff < 0) ? -rawDiff : rawDiff;
					int32_t allowDiff = calcDynamicThresholdPulse();	// 動的に算出した許容誤差
					bool canCorrect = isCross || (absDiff <= allowDiff);	// クロスは即補正、それ以外は閾値判定
					pathedMarker = clampMarkerIndex(nearestIdx);	// ヒント位置を更新
					if (canCorrect) {
						int32_t stepLimit = encMM(CORR_STEP_MAX_MM);	// 段階補正の上限量[パルス]
						int32_t diff = rawDiff;
						if (diff > stepLimit) {
							diff = stepLimit;	// 段階補正で切り詰め
						}
						if (diff < -stepLimit) {
							diff = -stepLimit;
						}
						int32_t errorDistance = encTotalOptimal - DistanceOptimal;	// 補正前の距離誤差を保持
						encTotalOptimal -= diff;	// 実距離を段階補正
						DistanceOptimal = encTotalOptimal - errorDistance;	// 誤差を維持したまま目標距離を更新
						int32_t markerIndex = markerPos[nearestIdx].indexPPAD;	// PPAD側の対応インデックス
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
						straightState = false;	// 多重補正防止
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
#ifndef LOG_RUNNING_WRITE
		// マーカーの位置を記録
		if (courseMarker == 0 && beforeCourseMarker > 0) {
			writeMarkerPos(encTotalOptimal, beforeCourseMarker);
		}
#endif
	}
	beforeCourseMarker = courseMarker; // 前回のマーカー状態を更新
	beforeModeCurve = modeCurve;       // 前回のカーブモードを更新
}

