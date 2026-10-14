#ifndef SLIP_DETECTION_H_
#define SLIP_DETECTION_H_
#include <stdbool.h>
#include <stdint.h>

#define SLIP_DETECTOR_LEGACY 0
#define SLIP_DETECTOR_SHADOW 1
#ifndef ROBOTRACE_SLIP_DETECTOR
#define ROBOTRACE_SLIP_DETECTOR SLIP_DETECTOR_SHADOW
#endif
#define SLIP_V2_VERSION 2U
#define SLIP_LAT_CANDIDATE_VERSION 1U
#define SLIP_V2_READY_SAMPLES (SLIP_V2_FILTER_MS + SLIP_V2_SLOW_MS + SLIP_V2_ENCODER_YAW_DELAY_MS)
#define SLIP_V2_FAST_MS 10U
#define SLIP_V2_SLOW_MS 30U
#define SLIP_V2_FILTER_MS 5U
#define SLIP_V2_ACCEL_DELAY_MS 0U
#define SLIP_V2_GYRO_DELAY_MS 0U
#define SLIP_V2_ENCODER_YAW_DELAY_MS 8U
#define SLIP_V2_HISTORY 64U
#define SLIP_V2_PULSE_PER_METER 58019.0F
#define SLIP_V2_LONG_ON 10.0F
#define SLIP_V2_LONG_OFF 8.0F
#define SLIP_V2_LAT_ON 4.6F
#define SLIP_V2_LAT_OFF 3.8F
#define SLIP_V2_YAW_ON_DPS 60.0F
#define SLIP_V2_YAW_OFF_DPS 45.0F
#define SLIP_V2_ON_MS 3U
#define SLIP_V2_OFF_MS 5U
// 13074～13083の保存モデル。物理トレッドではなく診断用経験校正。
#define SLIP_V2_YAW_WIDTH_M 0.11145811347181861F
#define SLIP_V2_YAW_OFFSET_MPS 0.0010439549422674179F
#define SLIP_V2_YAW_SCALE_DIFF 0.00717649365353785F
#define SLIP_V2_RAD_TO_DEG 57.29577951308232F

enum { SLIP_V2_LONG = 1U, SLIP_V2_LAT = 2U, SLIP_V2_YAW = 4U };
enum {
    SLIP_V2_DISABLED = 1U, SLIP_V2_NOT_RUNNING = 2U,
    SLIP_V2_CALIBRATING = 4U, SLIP_V2_IMU_ERROR = 8U,
    SLIP_V2_NONFINITE = 16U, SLIP_V2_LOW_SPEED = 32U,
    SLIP_V2_WARMUP = 64U, SLIP_V2_TIME_GAP = 128U,
    SLIP_V2_TANGENTIAL_UNAVAILABLE = 256U
};

typedef struct {
    int16_t pulseL, pulseR, pwmL, pwmR;
    float forwardMps2, lateralMps2, yawDps, currentL, currentR;
    uint32_t timeMs, gyroTimeMs, accelTimeMs, currentSequence, currentTimeMs;
    bool running, calibrating, gyroOk, accelOk, currentOk;
    float yawAlphaRadps2, tangentialAppliedXMps2;
    bool tangentialOk;
} SlipDetectionInput;

typedef struct {
    float longFast, longSlow, lateral, yawDps;
    uint8_t flags, valid, currentValid;
    uint16_t reason, currentHighSamples, pwmSum;
} SlipDetectionResult;

typedef struct {
    float peakLong, peakLat, peakYawDps;
    uint16_t onLongMs, onLatMs, onYawMs, validMs, totalMs, invalidReason;
} SlipDetectionInterval;

typedef struct { float v, a, lateral, yawRad, wheelDiff; } SlipDetectionFrame;
typedef struct {
    float residual, alphaRadps2, tangentialAppliedXMps2;
    uint8_t flag, valid;
    uint16_t reason;
} SlipLateralCandidateResult;
typedef struct {
    float peak;
    uint16_t onMs, validMs, invalidReason;
} SlipLateralCandidateInterval;
typedef struct {
    float filter[SLIP_V2_FILTER_MS], history[SLIP_V2_SLOW_MS];
    uint16_t filterIndex, filterCount, historyIndex, historyCount, sampleCount;
    uint8_t highMs, lowMs;
    SlipLateralCandidateResult result;
    SlipLateralCandidateInterval interval;
} SlipLateralCandidateState;
typedef struct {
    SlipDetectionFrame filter[SLIP_V2_FILTER_MS], history[SLIP_V2_HISTORY];
    uint16_t filterIndex, filterCount, historyIndex, historyCount;
    uint8_t highMs[3], lowMs[3];
    uint32_t previousTime, currentSequence;
    bool timeStarted, currentStarted;
    SlipDetectionResult result;
    SlipDetectionInterval interval;
    SlipLateralCandidateState lateralCandidate;
} SlipDetectionState;

extern SlipDetectionState slipShadow;
void SlipDetection_Reset(SlipDetectionState *state);
void SlipDetection_Update(SlipDetectionState *state, const SlipDetectionInput *input);
void SlipDetection_ResetInterval(SlipDetectionState *state);
float SlipDetection_LateralCandidateMps2(float currentMps2, float appliedTangentialMps2);
#endif
