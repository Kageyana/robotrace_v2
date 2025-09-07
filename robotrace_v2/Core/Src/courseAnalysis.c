//====================================//
// インクルード
//====================================//
#include "courseAnalysis.h"
#include "fatfs.h"
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
	float dl, drad, ret;

	const float invPulseConst = 10.0F / PALSE_MILLIMETER; // パルス→距離変換係数
	const float angFactor = DEG2RAD * dt; // 角速度→ラジアン変換係数
	dl = (float)velo * invPulseConst; // [palse] → [mm]
	drad = angvelo * angFactor; // [deg/s] → [rad]

	// 角速度が極小の場合は直線とみなして即座に返す
	if (fabsf(drad) < 1e-6F)
	{
		return 2000.0F;
	}

	ret = dl / drad; // 曲率半径を計算
	// 曲率半径が大きい＝直線の場合は極大にする
	if (fabsf(ret) > 1500.0F)
	{
		ret = 2000.0F;
	}

	return ret;
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
		f_printf(&fil, "%04d", fileNumber);
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
		f_gets(log, sizeof(log), &fil);
		sscanf(log, "%d", &analizedNumber);
		f_close(&fil);
	}

	for (int16_t i = 0; i <= endFileIndex; i++)
	{
		if (analizedNumber == fileNumbers[i])
		{
			fileIndexLog = i;
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readLogDistance
// 処理概要     距離基準2次走行の解析
// 引数         ログ番号(ファイル名)
// 戻り値       最適速度配列の最大要素数
//              -1: 解析用配列のサイズを超過
//              -2: メモリ確保失敗
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

	snprintf(fileName, sizeof(fileName), "%d", logNumber);			   // 数値を文字列に変換
	strcat(fileName, ".csv");										   // 拡張子を追加
	fresult = f_open(&fil_Read, fileName, FA_OPEN_EXISTING | FA_READ); // csvファイルを開く

	if (fresult == FR_OK)
	{
		// ログデータの取得
		TCHAR log[512];
		int32_t time, marker, velo, distance, roc, i = 0;
		float angVelo;
		int32_t numD = 0, numM = 0, cntCurR = 0, numStraight = 0;
		static int16_t ROCbuff[600] = {0};
		static int16_t *sortROC;
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
				sortROC = (int16_t *)malloc(sizeof(int16_t) * cntCurR); // 計算した曲率半径カウント分の配列を作成
				// メモリ確保の結果をチェックし、失敗時はエラー処理
				if (!sortROC) { // メモリ確保失敗時の処理
					printf("sortROC memory allocation error\n"); // エラーメッセージ表示
					ret = -2;            // エラーコード設定
					break;               // 未確保のため解放不要、ループを抜ける
				}
				memcpy(sortROC, ROCbuff, sizeof(int16_t) * cntCurR);    // 作成した配列に曲率半径をコピーする
				qsort(sortROC, cntCurR, sizeof(int16_t), cmpint16_t);   // ソート

				// 曲率半径を記録する
				if (cntCurR % 2 == 0)
				{
					// 中央値を記録(配列要素数が偶数のとき) 中央2つの平均値
					PPAD[numD].ROC = (sortROC[cntCurR / 2] + sortROC[cntCurR / 2 - 1]) / 2;
				}
				else
				{
					// 中央値を記録(配列要素数が奇数のとき)
					PPAD[numD].ROC = sortROC[cntCurR / 2];
				}
				free(sortROC); // mallocで確保したメモリを開放

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
					return -1; // 解析用配列のサイズを超えたら強制終了
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

		// インデックスが1多くなるので調整
		numM--;
		numD--;

		// 目標速度配列の整形 加減速が間に合うように距離を調整する
		float acceleration, elapsedTime, dv, dl;

		// 最初の要素は調整しない
		dl = (float)CALCDISTANCE / 1000;

		// 加速 インデックス2から開始
		for (i = 2; i <= numD; i++)
		{
			dv = (PPAD[i].boostSpeed - PPAD[i - 1].boostSpeed);
			elapsedTime = fabs(dl / dv);
			acceleration = dv / elapsedTime;
			if (acceleration > MACHINEACCELE)
			{
				PPAD[i].boostSpeed = PPAD[i - 1].boostSpeed + (MACHINEACCELE * dl);
			}
		}

		// 減速 インデックス末尾から開始
		for (i = numD - 1; i >= 1; i--)
		{
			dv = (PPAD[i].boostSpeed - PPAD[i + 1].boostSpeed);
			elapsedTime = fabs(dl / dv);
			acceleration = dv / elapsedTime;
			if (acceleration > MACHINEDECREACE)
			{
				PPAD[i].boostSpeed = PPAD[i + 1].boostSpeed + (MACHINEDECREACE * dl);
			}
		}

		for (i = 0; i <= numD; i++)
		{
			printf("%f\n", PPAD[i].boostSpeed);
		}

		numPPAMarry = numM;
		numPPADarry = numD;
		ret = numD;
	}
	else
	{
		ret = -4;
	}
	f_close(&fil_Read);

	// printf("Analysis distance end\n");

	// 解析済みのログ番号を保存
	saveLogNumber(logNumber);
	analizedNumber = logNumber;

	// 2次走行フラグ 距離基準2次走行
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
// モジュール名 cmpint16_t
// 処理概要     int16_t型の比較
// 引数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
int cmpint16_t(const void *n1, const void *n2)
{
	if (*(int16_t *)n1 > *(int16_t *)n2)
		return 1;
	else if (*(int16_t *)n1 < *(int16_t *)n2)
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
// モジュール名 processMarkerEvent
// 処理概要     マーカー通過時の処理をまとめる
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void processMarkerEvent(void) {
	// カーブマーカー,クロスラインを通過した時の処理
	if (courseMarker == 0 && beforeCourseMarker > 0) {
		cntMarker++; // マーカーカウント
		if (optimalTrace == BOOST_DISTANCE) {
			if (straightState) {
				// 距離基準2次走行かつストレート区間中のとき
				
				int32_t i, j, errorDistance = 0;
				for (i = pathedMarker; i <= numPPAMarry; i++) {
					// 現在地から一番近いマーカーを探す
					if (encTotalOptimal - markerPos[i].distance < 0) {
						for (j = i; j > 0; j--) {
							if (abs(encTotalOptimal - markerPos[j].distance) < encMM(100)) {
								errorDistance = encTotalOptimal - DistanceOptimal; // 現在の差を計算
								encTotalOptimal = markerPos[j].distance;	   // 距離を補正
								DistanceOptimal = encTotalOptimal - errorDistance; // 補正後の現在距離からの差分
								optimalIndex = markerPos[j].indexPPAD;		   // インデックス更新
								if (j - 5 < 0) {
									pathedMarker = j - 5;
								} else {
									pathedMarker = 0;
								}
								straightState = false;
								straightMeter = 0;
								break;
							}
						}
						if (errorDistance != 0)
						{
							break;
						}
					}
				}
			}
		} else if(optimalTrace == BOOST_SHORTCUT) {
			// ショートカット基準2次走行のとき
			
		}

		// マーカーの位置を記録
		if (courseMarker == 0 && beforeCourseMarker > 0) {
			writeMarkerPos(encTotalOptimal, beforeCourseMarker);
		}

		beforeCourseMarker = courseMarker; // 前回のマーカー状態を更新
	}
}
