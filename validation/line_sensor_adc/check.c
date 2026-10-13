#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define __IO volatile
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
#define HAL_TIM_ACTIVE_CHANNEL_1 1U
#define HAL_TIM_ACTIVE_CHANNEL_3 3U
#define TIM_IT_CC1 2U
#define TIM_FLAG_UPDATE 1U
#define TIM_FLAG_CC3 8U
#define HAL_ADC_STATE_BUSY_REG 0x100U
#define HAL_LOCKED 1U
#define HAL_DMA_STATE_READY 0U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define SidesensorL_GPIO_Port 1U
#define SidesensorR_GPIO_Port 2U
#define SidesensorL_Pin 1U
#define SidesensorR_Pin 1U
#define SidemarkerPWR_GPIO_Port 3U
#define SidemarkerPWR_Pin 1U
#define RIGHTMARKER 1U
#define LEFTMARKER 2U
#define MCU_VOLTAGE 3.3f
#define ADC1 1U
#define ADC2 2U
#define TIM1 ((MockTimer *)1)
#define TIM3 (&timerRegisters)
#define TIM6 ((MockTimer *)6)
#define TIM7 ((MockTimer *)7)
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR, DIER, CNT, ARR, CCR[4]; } MockTimer;
typedef struct { MockTimer *Instance; struct { uint32_t Period, Prescaler; } Init; uint32_t Channel; } TIM_HandleTypeDef;
typedef struct { uint32_t State; } DMA_HandleTypeDef;
typedef struct { uint32_t Instance, State, Lock, ErrorCode; DMA_HandleTypeDef *DMA_Handle; } ADC_HandleTypeDef;
static MockTimer timerRegisters;
static DMA_HandleTypeDef dma;
static TIM_HandleTypeDef htim3 = { &timerRegisters, {3749U, 11U}, 0U };
static ADC_HandleTypeDef hadc1 = { ADC1, 0U, 0U, 0U, &dma };
static uint16_t analogValLSon[11], analogValLSoff[11];
static uint8_t markerSensor;
static float adcVref = MCU_VOLTAGE;
static volatile uint16_t mockVrefCal = 1500U;
static bool datasentflag;
static uint32_t gpioLeft = 1, gpioRight = 1, markerPower, starts;
static HAL_StatusTypeDef nextStatus;
static uint32_t mockMask;
static uint32_t mockTick;
static uint32_t HAL_GetTick(void) { return mockTick; }
extern uint16_t lSensor[10], lSensorCari[10];
static void (*beforeLock)(void), (*afterStart)(void);
static bool inspectPublication, pendingReader;
static uint16_t oldRaw[10], oldNorm[10], newRaw[10], newNorm[10];
static uint32_t oldSequence, newSequence, blockedReads, readerCalls;
static void readAsTim6(void);

#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
#define __HAL_TIM_GET_COMPARE(t,c) ((t)->Instance->CCR[c])
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[c] = (uint32_t)(v))
#define __HAL_TIM_DISABLE_IT(t,f) ((t)->Instance->DIER &= ~(f))
#define __HAL_TIM_ENABLE_IT(t,f) ((t)->Instance->DIER |= (f))
#define __HAL_TIM_CLEAR_IT(t,f) ((t)->Instance->SR &= ~(f))
#define __HAL_TIM_DISABLE_OCxPRELOAD(t,c) ((void)(t), (void)(c))
static uint32_t __get_PRIMASK(void) {
    if (beforeLock) { void (*hook)(void) = beforeLock; beforeLock = NULL; hook(); }
    return mockMask;
}
static void __disable_irq(void) { mockMask = 1; }
static void __DMB(void) {}
static void __set_PRIMASK(uint32_t value) {
    mockMask = value;
    if (!value && pendingReader) { pendingReader = false; readAsTim6(); }
}
static void *mockMemcpy(void *dst, const void *src, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        ((unsigned char *)dst)[i] = ((const unsigned char *)src)[i];
        if (inspectPublication && (dst == lSensor || dst == lSensorCari)) {
            if (mockMask) { pendingReader = true; ++blockedReads; }
            else readAsTim6();
        }
    }
    return dst;
}
#define memcpy mockMemcpy
static uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *handle) { return handle->State; }
static uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *handle) { return handle->ErrorCode; }
static uint32_t HAL_GPIO_ReadPin(uint32_t port, uint32_t pin) { (void)pin; return port == 1U ? gpioLeft : gpioRight; }
static void HAL_GPIO_WritePin(uint32_t port, uint32_t pin, uint32_t value) { (void)port; (void)pin; markerPower = value; }
static HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *handle, uint32_t *buffer, uint32_t count) {
    CHECK(count == 11U && mockMask == 0U);
    CHECK(buffer == (uint32_t *)analogValLSon || buffer == (uint32_t *)analogValLSoff);
    ++starts;
    handle->State |= HAL_ADC_STATE_BUSY_REG;
    if (afterStart) { void (*hook)(void) = afterStart; afterStart = NULL; hook(); }
    return nextStatus;
}
static void getADC2(void) {}
static void Interrupt1ms(void) {}
static void Interrupt500us(void) {}
void getMarkerSensor(bool phase, uint8_t left, uint8_t right, uint32_t epoch);
void powerMarkerSensors(uint8_t onoff);
void getLineSensor(bool phase, const uint16_t *sample, uint32_t epoch);
#include "production.inc"
#undef memcpy

static void readAsTim6(void) {
    CHECK(!mockMask);
    bool old = memcmp(lSensor, oldRaw, sizeof(oldRaw)) == 0 &&
               memcmp(lSensorCari, oldNorm, sizeof(oldNorm)) == 0 && lineSensorUpdateCount == oldSequence;
    bool fresh = memcmp(lSensor, newRaw, sizeof(newRaw)) == 0 &&
                 memcmp(lSensorCari, newNorm, sizeof(newNorm)) == 0 && lineSensorUpdateCount == newSequence;
    CHECK(old || fresh);
    ++readerCalls;
}
static void reset(void) {
    beforeLock = afterStart = NULL;
    inspectPublication = false;
    mockMask = 0;
    mockTick = 0;
    timerRegisters = (MockTimer){ .ARR = 3749 };
    hadc1.State = hadc1.Lock = hadc1.ErrorCode = dma.State = 0;
    adcAcquisitionFault = adcAcquisitionBusy = false;
    adcSkippedCount = adcStartErrorCount = adcTransferErrorCount = 0;
    lineSensorMaxUpdateAgeMs = lineSensorStaleCount = 0;
    nextStatus = HAL_OK;
    starts = 0;
    powerLineSensors(0);
    monitorLineSensorUpdate1ms(true);
    powerLineSensors(1);
    for (unsigned i = 0; i < 10; ++i) { lSensorMin[i] = 0; lSensorMax[i] = 4095; }
    modeCalLinesensors = 0;
}
static void edge(bool on) {
    timerRegisters.SR = 0;
    timerRegisters.CNT = on ? 0U : timerRegisters.CCR[TIM_CHANNEL_3];
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_3;
    if (on) HAL_TIM_PeriodElapsedCallback(&htim3);
    else HAL_TIM_PWM_PulseFinishedCallback(&htim3);
    CHECK(lineSensorState == on && markerPower == (uint32_t)on);
}
static void request(void) {
    timerRegisters.CNT = timerRegisters.CCR[TIM_CHANNEL_1];
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    HAL_TIM_OC_DelayElapsedCallback(&htim3);
    CHECK(!(timerRegisters.DIER & TIM_IT_CC1));
}
static void begin(bool on, unsigned base) {
    edge(on);
    CHECK(conversionPending);
    gpioLeft = on ? 0U : 1U;
    gpioRight = 1U;
    request();
    CHECK(adcAcquisitionBusy);
    CHECK(adcSample.phase == on);
    CHECK(adcSample.buffer == (on ? analogValLSon : analogValLSoff));
    for (unsigned i = 0; i < 10; ++i) adcSample.buffer[i] = (uint16_t)(base + i);
    adcSample.buffer[10] = 1500;
}
static void finish(void) {
    hadc1.State = 0; // HAL clears BUSY before calling the production callback.
    HAL_ADC_ConvCpltCallback(&hadc1);
    CHECK(!adcAcquisitionBusy);
}
static void sample(bool on, unsigned base) { begin(on, base); finish(); }
static void frame(unsigned on, unsigned off) { sample(true,on); sample(false,off); sample(true,on); sample(false,off); }
static void assertRaw(unsigned expected) {
    for (unsigned i = 0; i < 10; ++i) { CHECK(lSensor[i] == expected); CHECK(lSensorCari[i] == expected); }
}
static void phaseInterrupt(void) { edge(true); CHECK(adcAcquisitionBusy); }
static void competingRequest(void) { edge(true); request(); CHECK(adcAcquisitionBusy); }
static void lateStart(void) { timerRegisters.CNT = 3700; timerRegisters.SR |= TIM_FLAG_CC3; }
static void powerCycle(void) { powerLineSensors(0); powerLineSensors(1); }

int main(void) {
    CHECK(!isLineSensorCalibrationValid());
    finishLineSensorCalibration(); CHECK(!isLineSensorCalibrationValid());
    reset();
    uint32_t count = lineSensorUpdateCount;
    sample(true,1000); sample(false,3000); sample(true,1002);
    CHECK(lineSensorUpdateCount == count);
    sample(false,3004);
    assertRaw(2001); CHECK(lineSensorUpdateCount == count + 1);
    CHECK(markerSensor == LEFTMARKER);
    frame(3000,1000); assertRaw(0); // Existing OFF-ON polarity and clipping.
    puts("PASS 1: normal two-sample means, polarity, clipping and 1ms update sequence");

    reset(); sample(true,1000); sample(false,3000); sample(true,1000); begin(false,3000);
    beforeLock = phaseInterrupt;
    finish(); assertRaw(2000);
    CHECK(markerSensor == LEFTMARKER && conversionPending);
    puts("PASS 2: ON edge during OFF completion retains OFF buffer and GPIO");

    reset(); frame(1000,3000); count = lineSensorUpdateCount;
    sample(true,100); sample(false,300); sample(true,100); begin(false,300);
    uint32_t beforeStarts = starts;
    beforeLock = competingRequest;
    finish();
    CHECK(adcSkippedCount == 1 && starts == beforeStarts && lineSensorUpdateCount == count + 1);
    assertRaw(200); // 取得済みの完成フレームは後続要求の欠測で破棄しない。
    frame(2000,3000); assertRaw(1000); CHECK(lineSensorUpdateCount == count + 2);
    puts("PASS 3/4: processing reservation skips once without wait; fresh pairs recover");

    reset(); sample(true,900); sample(false,3900); count = lineSensorUpdateCount;
    nextStatus = HAL_ERROR; hadc1.ErrorCode = 42;
    edge(true); request();
    CHECK(adcStartErrorCount == 1 && adcLastStartStatus == HAL_ERROR && adcLastErrorCode == 42);
    CHECK(!adcAcquisitionBusy && adcAcquisitionFault);
    HAL_ADC_ConvCpltCallback(&hadc1);
    CHECK(lineSensorUpdateCount == count);
    edge(false); request(); CHECK(starts == 3);
    reset(); begin(true,800); hadc1.ErrorCode = 7; HAL_ADC_ErrorCallback(&hadc1);
    CHECK(!adcAcquisitionBusy && adcAcquisitionFault && adcTransferErrorCount == 1 && adcLastErrorCode == 7);
    puts("PASS 5: start/transfer errors release reservation, retain cause, reject stale DMA");

    reset(); begin(true,1000); gpioLeft = 1; gpioRight = 0; finish();
    begin(false,3000); gpioLeft = 0; gpioRight = 0; finish();
    CHECK(markerSensor == LEFTMARKER);
    puts("PASS 6: marker uses GPIO at ADC start, preserving left/right and polarity");

    reset(); sample(true,100); sample(false,4000); begin(true,100);
    powerLineSensors(0); CHECK(!conversionPending && !markerPower);
    uint32_t savedStarts = starts; request(); CHECK(starts == savedStarts);
    powerLineSensors(1); finish();
    count = lineSensorUpdateCount; frame(2000,2500); assertRaw(500); CHECK(lineSensorUpdateCount == count + 1);
    reset(); sample(true,50); sample(false,4000); sample(true,50); begin(false,4000);
    beforeLock = powerCycle; count = lineSensorUpdateCount; finish();
    CHECK(lineSensorUpdateCount == count && lineSensorVrefSample == 0);
    frame(2000,2500); assertRaw(500);
    puts("PASS 7: power OFF/ON cancels CC1 and excludes averages and completions from old epoch");

    reset(); count = lineSensorUpdateCount;
    for (unsigned i = 0; i < 4; ++i) {
        bool on = (i & 1U) == 0U;
        edge(on); uint32_t epoch = acquisitionEpoch; powerLineSensors(1);
        CHECK(acquisitionEpoch == epoch && conversionPending && (timerRegisters.DIER & TIM_IT_CC1));
        request();
        for (unsigned j = 0; j < 10; ++j) adcSample.buffer[j] = on ? 1000 : 3000;
        powerLineSensors(1); CHECK(acquisitionEpoch == epoch && adcAcquisitionBusy);
        finish();
    }
    CHECK(lineSensorUpdateCount == count + 1); assertRaw(2000);
    puts("PASS: repeated power-ON requests from UI retain pending conversion and averaging");

    reset(); for (unsigned i = 0; i < 8; ++i) monitorLineSensorUpdate1ms(true);
    CHECK(!lineSensorUpdateStale && lineSensorUpdateAgeMs == 0);
    frame(1000,3000); monitorLineSensorUpdate1ms(true);
    for (unsigned i = 0; i < LS_UPDATE_STALE_MS; ++i) monitorLineSensorUpdate1ms(true);
    CHECK(lineSensorUpdateStale && lineSensorUpdateAgeMs == LS_UPDATE_STALE_MS && lineSensorStaleCount == 1);
    monitorLineSensorUpdate1ms(false); CHECK(!lineSensorUpdateStale);
    monitorLineSensorUpdate1ms(true); CHECK(lineSensorUpdateStale);
    frame(1000,3000); monitorLineSensorUpdate1ms(true);
    CHECK(!lineSensorUpdateStale && lineSensorUpdateAgeMs == 0 && lineSensorMaxUpdateAgeMs >= LS_UPDATE_STALE_MS);
    powerLineSensors(0); monitorLineSensorUpdate1ms(true); CHECK(!lineSensorUpdateStale);
    puts("PASS 8: 3ms diagnostic, recovery, startup/power/mode exclusions");

    reset(); frame(1000,3000);
    memcpy(oldRaw,lSensor,sizeof(oldRaw)); memcpy(oldNorm,lSensorCari,sizeof(oldNorm));
    oldSequence = lineSensorUpdateCount; newSequence = oldSequence + 1;
    for (unsigned i = 0; i < 10; ++i) newRaw[i] = newNorm[i] = 1000;
    sample(true,2000); sample(false,3000); sample(true,2000); begin(false,3000);
    blockedReads = readerCalls = 0; beforeLock = readAsTim6; inspectPublication = true;
    finish(); inspectPublication = false;
    CHECK(blockedReads == 40 && readerCalls >= 2); assertRaw(1000);
    puts("PASS 9: simulated TIM6 at every published byte sees only complete old/new frame");

    volatile float nominal = MCU_VOLTAGE;
    reset(); adcVref = nominal; getVref(); CHECK(adcVref == nominal);
    begin(true,1000); adcSample.buffer[10] = 0; finish(); getVref(); CHECK(adcVref == nominal);
    begin(false,3000); adcSample.buffer[10] = 1650; finish(); getVref(); CHECK(fabsf(adcVref - 3.0f) < 0.00001f);
    lineSensorVrefSample = 4096; getVref(); CHECK(fabsf(adcVref - 3.0f) < 0.00001f);
    puts("PASS 10: VREFINT missing/zero/out-of-range holds last valid reference");

    reset(); edge(true); CHECK(timerRegisters.CCR[0] == 900);
    request();
    finish(); edge(false); CHECK(timerRegisters.CCR[0] == 2774);
    timerRegisters.CNT = 3700; htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    HAL_TIM_OC_DelayElapsedCallback(&htim3); CHECK(!adcAcquisitionBusy && !conversionPending);
    savedStarts = starts; timerRegisters.CNT = 2774; HAL_TIM_OC_DelayElapsedCallback(&htim3); CHECK(starts == savedStarts);
    edge(true); timerRegisters.CNT = 900; timerRegisters.SR = TIM_FLAG_UPDATE;
    HAL_TIM_OC_DelayElapsedCallback(&htim3); CHECK(starts == savedStarts);
    timerRegisters.SR = 0; timerRegisters.CNT = 1800; lineSensorPhaseChanged(true); CHECK(!conversionPending);
    frame(1000,3000); assertRaw(2000);
    reset(); edge(true); afterStart = lateStart; request(); finish(); CHECK(lineSensorVrefSample == 0);
    frame(1000,3000); assertRaw(2000);
    puts("PASS: late CC1/edge/wrap and start preemption discard only expired data; 120us timing retained");

    reset(); edge(true); hadc1.State = HAL_ADC_STATE_BUSY_REG; request(); CHECK(adcSkippedCount == 1 && starts == 0);
    hadc1.State = 0; edge(false); hadc1.Lock = HAL_LOCKED; request(); CHECK(adcSkippedCount == 2 && starts == 0);
    hadc1.Lock = 0; edge(true); dma.State = 1; request(); CHECK(adcSkippedCount == 3 && starts == 0);
    dma.State = 0; frame(1000,3000); assertRaw(2000);
    puts("PASS: expected HAL/DMA/lock contention counted as skips, no fault latch");
    reset(); mockMask = 1; powerLineSensors(0); CHECK(mockMask == 1); mockMask = 0;
    reset();
    memset(lSensor,0,sizeof(lSensor)); memset(lSensorCari,0,sizeof(lSensorCari));
    for (unsigned i = 0; i < 10; ++i) { lSensorMin[i] = (uint16_t)(i*11); lSensorMax[i] = (uint16_t)(3000+i*30); }
    finishLineSensorCalibration(); CHECK(isLineSensorCalibrationValid());
    uint32_t random = 1234;
    uint32_t normalSkips = adcSkippedCount;
    count = lineSensorUpdateCount;
    for (unsigned n = 0; n < 4000; ++n) {
        bool on = (n & 1U) == 0U;
        edge(on); request();
        for (unsigned i = 0; i < 10; ++i) {
            random = random * 1664525U + 1013904223U;
            adcSample.buffer[i] = (uint16_t)((random >> 16) & 4095U);
        }
        adcSample.buffer[10] = 1500;
        legacyGetLineSensor(); finish();
        CHECK(memcmp(lSensor,legacyRaw,sizeof(legacyRaw)) == 0);
        CHECK(memcmp(lSensorCari,legacyNorm,sizeof(legacyNorm)) == 0);
        motorControlTrace(); legacyMotorControlTrace();
        CHECK(lineTraceCtrl.pwm == legacyCtrl.pwm && lineTraceCtrl.Int == legacyCtrl.Int);
        if ((n & 3U) == 3U) {
            CHECK(lineSensorUpdateCount == ++count);
            monitorLineSensorUpdate1ms(true);
        }
    }
    CHECK(adcSkippedCount == normalSkips && lineSensorMaxUpdateAgeMs == 1);
    puts("PASS regression: 4000 scans/1000 frames match original averages, normalization and PID output; 1ms updates");
    lSensorMax[9] = lSensorMin[9]; finishLineSensorCalibration(); CHECK(!isLineSensorCalibrationValid());
    lSensorMax[9] = 4096; finishLineSensorCalibration(); CHECK(!isLineSensorCalibrationValid());
    lSensorMax[9] = 4095; finishLineSensorCalibration(); CHECK(isLineSensorCalibrationValid());
    puts("PASS regression: valid/invalid RAM calibration completion");
    const unsigned missingMasks[] = {1U,2U,4U,8U,3U,6U,12U,9U};
    for (unsigned pattern = 0; pattern < sizeof(missingMasks)/sizeof(missingMasks[0]); ++pattern) {
        reset(); count = lineSensorUpdateCount;
        for (unsigned n = 0; n < 100; ++n) {
            mockTick = n;
            for (unsigned slot = 0; slot < 4; ++slot) {
                bool on = (slot & 1U) == 0U;
                if (missingMasks[pattern] & (1U << slot)) {
                    edge(on); hadc1.Lock = HAL_LOCKED; request(); hadc1.Lock = 0;
                } else sample(on,on ? 1000 : 3000);
            }
        }
        CHECK(lineSensorUpdateCount >= count + 49);
        assertRaw(2000);
    }
    puts("PASS: all single/adjacent-double periodic missing phases retain updates in 100ms");
    for (unsigned missingMask = 5U; missingMask <= 10U; missingMask += 5U) {
        reset(); frame(1000,3000); monitorLineSensorUpdate1ms(true); count = lineSensorUpdateCount;
        for (unsigned n = 0; n < 10; ++n) {
            mockTick = n + 1;
            for (unsigned slot = 0; slot < 4; ++slot) {
                bool on = (slot & 1U) == 0U;
                if (missingMask & (1U << slot)) {
                    edge(on); hadc1.Lock = HAL_LOCKED; request(); hadc1.Lock = 0;
                } else sample(on,on ? 2000 : 2500);
            }
            monitorLineSensorUpdate1ms(true);
        }
        CHECK(lineSensorUpdateCount == count && lineSensorUpdateStale);
    }
    puts("PASS: missing an entire optical phase produces no fabricated updates and remains diagnosed");
    reset(); sample(true,100); sample(false,4000); count = lineSensorUpdateCount;
    mockTick = LS_AVERAGE_MAX_PAIR_AGE_MS + 1;
    sample(true,2000); sample(false,3000); CHECK(lineSensorUpdateCount == count);
    sample(true,2000); sample(false,3000); CHECK(lineSensorUpdateCount == count + 1); assertRaw(1000);
    puts("PASS: expired complete pairs are excluded, preserving freshness and two samples per phase");
    puts("All ADC phase tests passed");
    return 0;
}
