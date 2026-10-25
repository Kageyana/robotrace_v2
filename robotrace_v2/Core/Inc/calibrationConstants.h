#ifndef CALIBRATION_CONSTANTS_H_
#define CALIBRATION_CONSTANTS_H_

// 実測で検証済みの校正基準。実機換算の既定値とログ受入条件で共有する。
// 変更時は実測校正と経路元ログの受入条件を再検証する。
#define CALIBRATED_ENCODER_PULSE_PER_METER 58019 // 左右平均、動力1m×5走[pulse/m]
#define CALIBRATED_IMU_SAMPLE_COUNT 100U // 有効なジャイロ・加速度校正サンプル数

#endif // CALIBRATION_CONSTANTS_H_
