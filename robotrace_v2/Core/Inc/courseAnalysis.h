#ifndef COURSEANALYSIS_H_
#define COURSEANALYSIS_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "encoder.h"
#include "stdlib.h"
#include "math.h"
//====================================//
// シンボル定義
//====================================//
#define OPT_BUFF_SIZE 1000
#define CALCDISTANCE 50				// 距離解析ステップ[mm]
#define CALCDISTANCE_SHORTCUT 10 	// 距離解析ステップ(ショートカット走行)[mm]
#define MACHINEACCELE 3.4F			// 加速度[m/s^2]
#define MACHINEDECREACE 3.0F		// 減速度[m/s^2]

#define BOOST_NONE 0				// 1次走行
#define BOOST_MARKER 1				// マーカー基準2次走行
#define BOOST_DISTANCE 2			// 距離基準2次走行
#define BOOST_SHORTCUT 3			// ショートカット2次走行
#define BOOST_PATH_REPLAY 4			// 一次走行経路のセンサレス再走行

#define SEARCHRANGE 150				// 距離補正時の距離検索範囲[mm]

#define SHORTCUTWINDOW 4			// ショートカットコース生成時の移動平均サンプル数

#define ROC_STRAIGHT_TH 2800.0F		// 直線とみなす曲率半径の閾値[mm]
#define ROC_STRAIGHT_MAX 3000.0F		// 直線とみなす曲率半径の閾値[mm]

// 3次走行用スリップ解析(2次ログ)の調整用定数
#define CA_SECOND_LOG_LINE_BUFSIZE 1600		// 2次ログ1行バッファサイズ
#define CA_SLIP_CNT_MIN 3				// スリップ回数のノイズ除外閾値(値↑で判定が厳しくなりリスク↓→減速弱)
#define CA_SLIP_FRAC_FULL 0.60f			// risk=1.0とみなすスリップ割合(値↑でフルリスク到達しにくく減速弱、例:0.60→0.70)
#define CA_SLIP_EXPAND_1 0.50f			// 近傍拡張係数(±1)(値↑で周辺にもリスク拡散→減速範囲広、例:0.25→0.40)
#define CA_SLIP_EXPAND_2 0.25f			// 近傍拡張係数(±2)(値↑で遠方へも拡散→減速範囲広、例:0.25→0.40)
#define CA_SLIP_DOWN_RISK 0.20f			// riskに応じた基本減速ゲイン(値↑で減速強、例:0.20→0.30)
#define CA_SLIP_DOWN_LONG_EXTRA 0.01f	// 縦スリップ追加減速(値↑で縦スリップ時の減速強、例:0.01→0.02)
#define CA_SLIP_DOWN_LAT_EXTRA 0.3f		// 横スリップ追加減速(値↑で横スリップ時の減速強、例:0.10→0.15)
#define CA_SLIP_MIN_SCALE 0.80f			// 最小スケール(減速下限)(値↑で下限が高くなり減速弱、値↓で減速強)
#define CA_SLIP_UP_STRAIGHT 0.05f		// 直線での微増速(値↑で増速強、例:0.09→0.12)
#define CA_SLIP_UP_CURVE 0.05f			// カーブでの微増速(値↑で増速強、例:0.05→0.08)

// 距離補正関連の定数
#define STRAIGHT_WINDOW_MM			150		// 直線判定に用いる距離窓[mm]
#define STRAIGHT_RATIO_THRESHOLD	0.80f	// 直線率の閾値
#define CORR_THRESH_MIN_MM			120		// 補正許容誤差の下限[mm]
#define CORR_THRESH_MAX_MM			200		// 補正許容誤差の上限[mm]
#define CORR_STEP_MAX_MM			200		// 1回あたりの最大補正量[mm]
#define FAILSAFE_MISS_MAX			2		// 補正失敗回数の閾値
#define FAILSAFE_SPEED_SCALE		0.85f	// フェイルセーフ時の速度倍率
#define MARKER_SEARCH_BACK			6		// マーカー探索時の後方探索幅
#define MARKER_SEARCH_FORWARD		6		// マーカー探索時の前方探索幅
#define MARKER_SEARCH_CROSS_BACK	14		// クロス時の後方探索幅
#define MARKER_SEARCH_CROSS_FORWARD	14		// クロス時の前方探索幅
#define MARKER_INDEX_DEV_NORMAL		24		// 通常補正時に許容するoptimalIndex近傍幅
#define MARKER_INDEX_DEV_CROSS		32		// クロス補正時に許容するoptimalIndex近傍幅
#define MARKER_INDEX_JUMP_CROSS_MAX	16		// クロス補正時の1回あたり最大indexジャンプ
#define CORR_DYN_COEFF_SPEED		0.02f	// 速度依存補正係数
#define CORR_DYN_COEFF_ANG			0.5f	// 角速度依存補正係数

// #define WRITE_BOOSTSPEED_LOG 	 // 速度計画ログを書き出すかどうかのフラグ

// ファイル名
#define FILENAME_ANALYSIS_NUMBER "analysis"

typedef struct
{
	int16_t ROC;
	float boostSpeed;
} AnalysisData;

typedef struct
{
	int32_t distance;
	int32_t indexPPAD;
} EventPos;

typedef struct
{
	float x;
	float y;
	float w;
} Courseplot;
//====================================//
// グローバル変数の宣言
//====================================//
// 2次走行関係
extern uint8_t optimalTrace;
extern uint16_t optimalIndex;
extern int16_t numPPADarry;
extern int16_t numPPAMarry;
extern int16_t indexSC;
extern int16_t pathedMarker;
extern float boostSpeed;
extern int32_t DistanceOptimal;
extern int16_t analyzedNumber;
extern int32_t encTotalOptimal;
extern int32_t encPID;
extern int32_t straightMeter;
extern bool straightState;
extern bool straightMarkerPending;
extern uint8_t straightMarkerPendingLog;

// 解析関係
extern AnalysisData PPAD[OPT_BUFF_SIZE];
extern EventPos markerPos[OPT_BUFF_SIZE];
extern Courseplot xycie;
//====================================//
// プロトタイプ宣言
//====================================//
float calcROC(int16_t velo, float angvelo, float dt);
void saveLogNumber(int16_t fileNumber);
void getLogNumber(void);
int16_t readLogDistance(int logNumber);
int16_t readLogDistanceSlip(int logNumber);
float asignVelocity(int16_t ROC);
int cmpfloat(const void *n1, const void *n2);
int16_t readLogTest(int logNumber);
int16_t calcXYcies(int logNumber);
void calcXYcie(int16_t encpulse, float angVelo, float dt);
void clearXYcie(void);
void processMarkerEvent(void);
void clearMarkerProcessState(void);

#endif // COURSEANALYSIS_H_
