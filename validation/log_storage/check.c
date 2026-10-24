// 実際の保存・読込・経路解析を、RAM上のFatFsと障害注入で検証する。
#include "SDcard.h"
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
#include "../../robotrace_v2/Core/Src/SDcard.c"
#include "../../robotrace_v2/Core/Src/courseAnalysis.c"
#include "../../robotrace_v2/Core/Src/pathFollower.c"
#include <assert.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)
#define MOCK_FILES 40
#define MOCK_CAPACITY (3U * 1024U * 1024U)
typedef struct { char name[40]; uint8_t *data; size_t size; bool used; } MockFile;
static MockFile files[MOCK_FILES];
static struct { FIL *handle; MockFile *file; BYTE mode; } handles[8];
static unsigned directoryPosition;
static const char *failureOperation, *failureExtension;
static unsigned failureSkip;
static bool shortTransfer;
static bool mockExpandDenied;
static uint32_t mockReadCalls, mockWriteCalls, mockReadMax, mockWriteMax;
static uint64_t mockReadBytes, mockWriteBytes;
static uint32_t mockTick;

bool modeLOG, initMSD = true;
uint8_t autoStart, SGmarker, emcStop, targetSpeed, courseMarkerLog, markerSensor, patternTrace;
uint32_t cntRun;
int32_t encRightMarker, log_targetAngularVelocity;
int16_t encCurrentN, encCurrentL, encCurrentR;
int32_t encTotalL, encTotalR;
uint16_t lSensorCari[NUM_SENSORS];
uint16_t lSensorMax[NUM_SENSORS], lSensorMin[NUM_SENSORS];
static bool mockLineSensorCalibrationValid;
float batteryVoltage_V = 7.8F, motorVoltageCmdL_V, motorVoltageCmdR_V;
float motorCurrentL, motorCurrentR;
volatile IMUval imuVal;
float angleOffset[3], imuTempCoeff_dpsPerC, imuTempCalibrationStart_C, imuTempCalibration_C;
float imuTempCalibrationEnd_C, imuTempEnd_C;
bool imuTempCalibrationValid, imuTempCorrectionEnabled;
uint16_t imuTempCalibrationSamples, imuTempCalibrationReadErrors;
pidParam lineTraceCtrl, lineTraceOmegaFBCtrl, veloCtrl, yawRateCtrl;
int16_t speedFeedForwardGain;
speedParam tgtParam;
AutoRunState autoRunState;
volatile DWORD g_sd_last_read_sector;
volatile UINT g_sd_last_read_count;
volatile int g_sd_last_read_blocks_status, g_sd_last_read_multi_status;

/////////////////////////////////////////////////////////////////////
// モジュール名 HAL_GetTick
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
uint32_t HAL_GetTick(void) { return mockTick++; }
/////////////////////////////////////////////////////////////////////
// モジュール名 updateBatteryVoltage
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void updateBatteryVoltage(void) {}
/////////////////////////////////////////////////////////////////////
// モジュール名 updateImuTempEndTemperature
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void updateImuTempEndTemperature(void) {}
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationReady
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool IMU_CalibrationReady(void) { return true; }
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationSamples
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
uint16_t IMU_CalibrationSamples(void) { return IMU_CALIBRATION_SAMPLE_COUNT; }
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationReadErrors
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
uint16_t IMU_CalibrationReadErrors(void) { return 0U; }
/////////////////////////////////////////////////////////////////////
// モジュール名 markerGetGoalOnsetPulse
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool markerGetGoalOnsetPulse(int32_t *pulse) { *pulse = encRightMarker; return encRightMarker > 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 calcDlMm
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float calcDlMm(int16_t speed, float dt) { return (float)speed / PULSE_MILLIMETER * dt * 1000.0F; }
/////////////////////////////////////////////////////////////////////
// モジュール名 setTargetAngularVelocity
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void setTargetAngularVelocity(float value) { (void)value; }
/////////////////////////////////////////////////////////////////////
// モジュール名 isLineSensorCalibrationValid
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool isLineSensorCalibrationValid(void) { return mockLineSensorCalibrationValid; }
/////////////////////////////////////////////////////////////////////
// モジュール名 sd_remount
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int sd_remount(void) { return FR_OK; }

// 未使用の周辺ハードウェア依存は固定値で代替する。
SPI_HandleTypeDef hspi3;
uint8_t courseMarker, beforeCourseMarker;
uint16_t cntMarker;
bool stateCrossLine;
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_SetTempCompensationCoefficient
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void IMU_SetTempCompensationCoefficient(int32_t value) { (void)value; }
/////////////////////////////////////////////////////////////////////
// モジュール名 HAL_GPIO_ReadPin
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) { (void)port; (void)pin; return GPIO_PIN_RESET; }
/////////////////////////////////////////////////////////////////////
// モジュール名 HAL_SPI_GetState
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
HAL_SPI_StateTypeDef HAL_SPI_GetState(const SPI_HandleTypeDef *spi) { (void)spi; return HAL_SPI_STATE_READY; }
/////////////////////////////////////////////////////////////////////
// モジュール名 sd_mount
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int sd_mount(void) { return FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_getfree
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_getfree(const TCHAR *name, DWORD *freeClusters, FATFS **fs) { (void)name; (void)freeClusters; (void)fs; return FR_DISK_ERR; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_mkdir
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_mkdir(const TCHAR *name) { (void)name; return FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipFlag
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool getSlipFlag(void) { return false; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipFlagLat
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool getSlipFlagLat(void) { return false; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipIndicatorRaw
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipIndicatorRaw(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipIndicatorFiltered
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipIndicatorFiltered(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipLatEnabled
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool getSlipLatEnabled(void) { return false; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipLatOnCountEnabled
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
bool getSlipLatOnCountEnabled(void) { return false; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipISumOkCount
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
uint16_t getSlipISumOkCount(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipPwmSumF
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipPwmSumF(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipISumF
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipISumF(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipEncAyF
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipEncAyF(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 getSlipImuAyF
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float getSlipImuAyF(void) { return 0; }
/////////////////////////////////////////////////////////////////////
// モジュール名 Control_GetEncCurrentCorr_p
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int32_t Control_GetEncCurrentCorr_p(void) { return encCurrentN; }
/////////////////////////////////////////////////////////////////////
// モジュール名 Control_ApplyMarkerCorrection_p
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void Control_ApplyMarkerCorrection_p(int32_t value) { (void)value; }
/////////////////////////////////////////////////////////////////////
// モジュール名 setTargetSpeed
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void setTargetSpeed(float speed) { (void)speed; }
/////////////////////////////////////////////////////////////////////
// モジュール名 resetSpeedPID
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
void resetSpeedPID(void) {}
/////////////////////////////////////////////////////////////////////
// モジュール名 encMM
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int32_t encMM(int16_t mm) { return (int32_t)(PULSE_MILLIMETER * abs(mm)); }
/////////////////////////////////////////////////////////////////////
// モジュール名 encPulse
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
float encPulse(int32_t pulse) { return (float)abs(pulse) / PULSE_MILLIMETER; }

/////////////////////////////////////////////////////////////////////
// モジュール名 findFile
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static MockFile *findFile(const char *name)
{
    for (unsigned i = 0; i < MOCK_FILES; i++) if (files[i].used && !strcmp(files[i].name, name)) return &files[i];
    return NULL;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 makeFile
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static MockFile *makeFile(const char *name)
{
    MockFile *file = findFile(name);
    if (file) return file;
    for (unsigned i = 0; i < MOCK_FILES; i++) if (!files[i].used) {
        file = &files[i]; file->used = true; file->size = 0;
        snprintf(file->name, sizeof(file->name), "%s", name);
        if (!file->data) file->data = calloc(1U, MOCK_CAPACITY);
        CHECK(file->data); return file;
    }
    CHECK(false); return NULL;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 handleIndex
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static unsigned handleIndex(FIL *file)
{
    for (unsigned i = 0; i < 8U; i++) if (handles[i].handle == file) return i;
    CHECK(false); return 0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 fault
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static bool fault(const char *operation, const char *name)
{
    if (!failureOperation || strcmp(operation, failureOperation) ||
        (failureExtension && !strstr(name, failureExtension))) return false;
    if (failureSkip) { failureSkip--; return false; }
    failureOperation = NULL;
    return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_open
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_open(FIL *file, const TCHAR *name, BYTE mode)
{
    if (fault("open", name)) return FR_DISK_ERR;
    MockFile *entry = findFile(name);
    if (entry && (mode & FA_CREATE_NEW)) return FR_EXIST;
    if (!entry && !(mode & (FA_CREATE_ALWAYS | FA_CREATE_NEW | FA_OPEN_ALWAYS))) return FR_NO_FILE;
    if (!entry) entry = makeFile(name);
    if (mode & FA_CREATE_ALWAYS) entry->size = 0;
    for (unsigned i = 0; i < 8U; i++) if (!handles[i].handle) {
        handles[i].handle = file; handles[i].file = entry; handles[i].mode = mode;
        memset(file, 0, sizeof(*file)); file->obj.objsize = (FSIZE_t)entry->size; return FR_OK;
    }
    return FR_TOO_MANY_OPEN_FILES;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_close
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_close(FIL *file)
{
    unsigned i = handleIndex(file);
    bool fail = fault("close", handles[i].file->name);
    handles[i].handle = NULL;
    return fail ? FR_DISK_ERR : FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_read
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_read(FIL *file, void *buffer, UINT requested, UINT *read)
{
    mockReadCalls++; mockReadBytes += requested;
    if (requested > mockReadMax) mockReadMax = requested;
    MockFile *entry = handles[handleIndex(file)].file;
    if (fault("read", entry->name)) {
        *read = 0U;
        if (!shortTransfer) { file->err = FR_DISK_ERR; return FR_DISK_ERR; }
        requested = requested ? requested - 1U : 0U;
    }
    size_t available = file->fptr < entry->size ? entry->size - file->fptr : 0;
    *read = (UINT)(requested < available ? requested : available);
    memcpy(buffer, entry->data + file->fptr, *read); file->fptr += *read;
    return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_write
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_write(FIL *file, const void *buffer, UINT requested, UINT *written)
{
    mockWriteCalls++; mockWriteBytes += requested;
    if (requested > mockWriteMax) mockWriteMax = requested;
    unsigned i = handleIndex(file);
    MockFile *entry = handles[i].file;
    CHECK(handles[i].mode & FA_WRITE);
    if (fault("write", entry->name)) {
        *written = 0U;
        if (!shortTransfer) return FR_DISK_ERR;
        requested = requested ? requested - 1U : 0U;
    }
    CHECK(file->fptr + requested <= MOCK_CAPACITY);
    memcpy(entry->data + file->fptr, buffer, requested);
    file->fptr += requested; if (file->fptr > entry->size) entry->size = file->fptr;
    file->obj.objsize = (FSIZE_t)entry->size; *written = requested; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_lseek
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_lseek(FIL *file, FSIZE_t offset)
{
    MockFile *entry = handles[handleIndex(file)].file;
    if (fault("seek", entry->name)) return FR_DISK_ERR;
    CHECK(offset <= MOCK_CAPACITY); file->fptr = offset; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_truncate
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_truncate(FIL *file)
{
    MockFile *entry = handles[handleIndex(file)].file;
    if (fault("truncate", entry->name)) return FR_DISK_ERR;
    entry->size = file->fptr; file->obj.objsize = file->fptr; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_sync
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_sync(FIL *file) { return fault("sync", handles[handleIndex(file)].file->name) ? FR_DISK_ERR : FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_expand
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_expand(FIL *file, FSIZE_t size, BYTE contiguous)
{
    (void)contiguous; MockFile *entry = handles[handleIndex(file)].file;
    if (fault("expand", entry->name)) return FR_DISK_ERR;
    if (mockExpandDenied) return FR_DENIED;
    CHECK(size <= MOCK_CAPACITY); entry->size = size; file->obj.objsize = size; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_puts
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int f_puts(const TCHAR *text, FIL *file)
{
    UINT written, length = 0U;
    uint8_t buffer[64];
    int total = 0;
    while (*text) {
        char c = *text++;
        if (c == '\n') {
            buffer[length++] = '\r';
            if (length == sizeof(buffer)) {
                if (f_write(file, buffer, length, &written) != FR_OK || written != length) return -1;
                total += (int)length; length = 0U;
            }
        }
        buffer[length++] = (uint8_t)c;
        if (length == sizeof(buffer)) {
            if (f_write(file, buffer, length, &written) != FR_OK || written != length) return -1;
            total += (int)length; length = 0U;
        }
    }
    if (length && (f_write(file, buffer, length, &written) != FR_OK || written != length)) return -1;
    return total + (int)length;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_gets
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
TCHAR *f_gets(TCHAR *line, int capacity, FIL *file)
{
    int i = 0;
    while (i < capacity - 1) {
        char byte; UINT read;
        if (f_read(file, &byte, 1U, &read) != FR_OK || read == 0U) break;
        if (byte == '\r') continue;
        line[i++] = byte; if (byte == '\n') break;
    }
    line[i] = 0; return i ? line : NULL;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_printf
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
int f_printf(FIL *file, const TCHAR *format, ...)
{
    char line[1024]; va_list args; va_start(args, format);
    int size = vsnprintf(line, sizeof(line), format, args); va_end(args);
    return size >= 0 && (size_t)size < sizeof(line) ? f_puts(line, file) : -1;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_stat
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_stat(const TCHAR *name, FILINFO *info)
{
    if (fault("stat", name)) return FR_DISK_ERR;
    MockFile *entry = findFile(name); if (!entry) return FR_NO_FILE;
    memset(info, 0, sizeof(*info)); info->fsize = (FSIZE_t)entry->size; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_unlink
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_unlink(const TCHAR *name)
{
    if (fault("unlink", name)) return FR_DISK_ERR;
    MockFile *entry = findFile(name); if (!entry) return FR_NO_FILE;
    entry->used = false; return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_rename
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_rename(const TCHAR *oldName, const TCHAR *newName)
{
    if (fault("rename", newName)) return FR_DISK_ERR;
    MockFile *entry = findFile(oldName); if (!entry) return FR_NO_FILE;
    if (findFile(newName)) return FR_EXIST;
    snprintf(entry->name, sizeof(entry->name), "%s", newName); return FR_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_opendir
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_opendir(DIR *dir, const TCHAR *name)
{ (void)dir; (void)name; directoryPosition = 0; return fault("opendir", "/") ? FR_DISK_ERR : FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_closedir
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_closedir(DIR *dir) { (void)dir; return fault("closedir", "/") ? FR_DISK_ERR : FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_readdir
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
FRESULT f_readdir(DIR *dir, FILINFO *info)
{
    (void)dir;
    if (fault("readdir", "/")) return FR_DISK_ERR;
    memset(info, 0, sizeof(*info));
    while (directoryPosition < MOCK_FILES) {
        MockFile *entry = &files[directoryPosition++];
        if (entry->used) { snprintf(info->fname, sizeof(info->fname), "%s", entry->name); info->fsize = entry->size; break; }
    }
    return FR_OK;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 resetMock
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static void resetMock(void)
{
    for (unsigned i = 0; i < MOCK_FILES; i++) files[i].used = false;
    memset(handles, 0, sizeof(handles));
    binarySource.file = NULL; logTempOpen = false; modeLOG = false;
    logRecoveryReady = true; sd_fatfs_locked = false; sd_analysis_active = false;
    logFileNumber = 0; cntSend = 0; failureOperation = failureExtension = NULL; failureSkip = 0; shortTransfer = false;
    patternTrace = 0; autoStart = 0; emcStop = 0; SGmarker = COUNT_GOAL; encRightMarker = 0;
    mockExpandDenied = false;
    optimalTrace = BOOST_NONE; autoRunState = (AutoRunState){0};
    tgtParam = (speedParam){0};
    memset(lSensorMax, 0, sizeof(lSensorMax));
    memset(lSensorMin, 0, sizeof(lSensorMin));
    mockLineSensorCalibrationValid = false;
    tgtParam.bstStraight = 2.0F; tgtParam.bst100 = 1.0F; tgtParam.acceleF = 5.0F; tgtParam.acceleD = 5.0F;
    tgtParam.pathReplay = 1.0F; tgtParam.shortCut = 1.0F;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 appendRecordRaw
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static void appendRecordRaw(uint16_t time, uint32_t pulse, int32_t left, int32_t right, float heading, uint8_t marker)
{
    CHECK(logBufferWriterBeginRecord(&logBufferWriter, LOG_SIZE));
#define LOG_SEND_U8(v) send8bit((uint8_t)(v))
#define LOG_SEND_U16(v) send16bit((uint16_t)(v))
#define LOG_SEND_S16(v) send16bit((uint16_t)(int16_t)(v))
#define LOG_SEND_U32(v) send32bit((uint32_t)(v))
#define LOG_SEND_S32(v) send32bit((uint32_t)(int32_t)(v))
#define LOG_SEND_F32(v) logSendFloat((float)(v))
#define FIELD(type, name, fmt, expr) LOG_SEND_##type(record.name);
#define SKIP(type, name, fmt, expr)
    LogRecord record = {0};
    record.cntlog = time; record.encCurrentN = 100; record.encCurrentCorr_p = 100;
    record.encTotalOptimal = pulse; record.imuAngle_Z = heading; record.courseMarker = marker;
    record.encTotalL = left; record.encTotalR = right;
    record.targetSpeed = 100; record.optimalIndex = 0;
    record.slipV2TotalMs = record.slipV2ValidMs = record.slipV2LatValidMs = 10U;
    record.slipV2OnLongMs = (time / 10U) % 7U < 3U ? 4U : 0U;
    record.slipV2OnLatMs = (time / 10U) % 11U < 2U ? 3U : 0U;
    record.gyroVal_Z = marker == 3U ? 180.0F : 0.0F;
    record.motorCurrentL = 1.125F; record.motorCurrentR = -0.25F; record.imuTemp_C = 31.125F;
    LOG_FIELD_LIST(FIELD, SKIP)
#undef FIELD
#undef SKIP
#undef LOG_SEND_U8
#undef LOG_SEND_U16
#undef LOG_SEND_S16
#undef LOG_SEND_U32
#undef LOG_SEND_S32
#undef LOG_SEND_F32
    CHECK(logBufferWriterEndRecord(&logBufferWriter)); cntSend++;
    writeLogPuts();
}
/////////////////////////////////////////////////////////////////////
// モジュール名 appendRecord
// 処理概要     マーカー補正なしの検証レコードを追加する
// 引数         time: 時刻[ms], pulse: 累積値[pulse], heading: 角度[deg], marker: マーカー値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void appendRecord(uint16_t time, uint32_t pulse, float heading, uint8_t marker)
{
    appendRecordRaw(time, pulse, (int32_t)pulse, (int32_t)pulse, heading, marker);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 saveRun
// 処理概要     ログ検証用の模擬処理または受入条件を実行する
// 引数         定義中の引数（テスト入力・模擬I/O）
// 戻り値       定義の型に従う検証値
/////////////////////////////////////////////////////////////////////
static uint16_t saveRun(unsigned rows, uint8_t stop)
{
    initLog(); CHECK(logTempOpen); modeLOG = true;
    for (unsigned i = 0; i < rows; i++) appendRecord((uint16_t)((i + 1U) * 10U), (i + 1U) * 1000U, 0.0F, i == 5U ? 3U : 0U);
    encRightMarker = (int32_t)(rows * 1000U); emcStop = stop;
    CHECK(endLog()); CHECK(!modeLOG && lastRunSaved);
    char csv[20], bin[20];
    logBinaryName(csv, sizeof(csv), logFileNumber, "csv"); logBinaryName(bin, sizeof(bin), logFileNumber, "bin");
    CHECK(!findFile(csv) && findFile(bin));
    return logFileNumber;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 savedMetadata
// 処理概要     実保存binのヘッダとメタデータCRCを照合する
// 引数         number: 保存ログ番号
// 戻り値       次の呼出しまで有効なメタデータ文字列
/////////////////////////////////////////////////////////////////////
static const char *savedMetadata(unsigned number)
{
    static char metadata[4096];
    char name[20]; logBinaryName(name,sizeof(name),(uint16_t)number,"bin");
    MockFile *bin=findFile(name); CHECK(bin);
    LogBinaryHeader h; CHECK(logBinaryDecodeHeader(bin->data,bin->size,&h));
    CHECK(h.recordBytes==LOG_SIZE && h.schema==logBinarySchema());
    const uint8_t *data=bin->data+LOG_BINARY_HEADER_BYTES+(size_t)h.rows*h.recordBytes;
    CHECK(~logBinaryCrcUpdate(UINT32_MAX,data,h.schemaBytes)==h.schema);
    data+=h.schemaBytes;
    CHECK(h.metadataBytes<sizeof(metadata));
    CHECK(~logBinaryCrcUpdate(UINT32_MAX,data,h.metadataBytes)==h.metadataCrc);
    memcpy(metadata,data,h.metadataBytes); metadata[h.metadataBytes]=0;
    return metadata;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testStorageAndSnapshots
// 処理概要     5走のbin保持・開始時校正値・再起動検証・採番を確認する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testStorageAndSnapshots(void)
{
    resetMock();
    for(unsigned run=0;run<5;run++) {
        for(unsigned i=0;i<NUM_SENSORS;i++) { lSensorMax[i]=(uint16_t)(4000-run*100-i); lSensorMin[i]=(uint16_t)(100+run*10+i); }
        mockLineSensorCalibrationValid=true;
        autoStart=(uint8_t)(run+1); CHECK(saveRun(30,run==4?3:0)==run+1);
    }
    memset(lSensorMax,0,sizeof(lSensorMax)); memset(runLineSensorMax,0,sizeof(runLineSensorMax));
    for(unsigned run=0;run<5;run++) {
        const char *meta=savedMetadata(run+1); char expected[80];
        snprintf(expected,sizeof(expected),"lSensorMax9=%u,",4000-run*100-9); CHECK(strstr(meta,expected));
        snprintf(expected,sizeof(expected),"lSensorMin0=%u,",100+run*10); CHECK(strstr(meta,expected));
        CHECK(strstr(meta,"lineSensorCalibrationValid=1,"));
        CHECK(strstr(meta,run==4?"emcStop=3.00,":"emcStop=0.00,"));
    }
    CHECK(calcNextLogNumber()==6 && logPrepareBinaryStorage() && logRecoveryIsReady());
    CHECK(!findFile("1.csv") && !findFile("1.part"));
    makeFile("6.tmp"); makeFile("10.csv"); makeFile("11.part");
    CHECK(logPrepareBinaryStorage() && logHasIncompleteTemp());
    CHECK(getFileNumbers()==4 && fileNumbers[0]==1 && fileNumbers[4]==5);
    CHECK(calcNextLogNumber()==12 && findFile("6.tmp") && findFile("10.csv"));
    CHECK(!logSourceInfo(NULL,&(LogBinaryHeader){0}));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testPlansAndReader
// 処理概要     型付き読込・DISTANCE/SLIP/PATH・キャッシュと元番号保持を確認する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testPlansAndReader(void)
{
    resetMock(); saveRun(100,0);
    FIL file; LogSourceRow row; LogBinaryHeader h;
    CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_OK && logSourceInfo(&file,&h));
    CHECK(h.rows==100 && strstr(logSourceMetadata(&file),"pathSourceFormatVersion=1,"));
    CHECK(logSourceOpen(&(FIL){0},1,LOG_SOURCE_PATH)==FR_LOCKED);
    unsigned count=0;
    while(logSourceNext(&file,&row)) {
        ++count; CHECK(row.cntlog==(int32_t)count*10 && row.encTotalOptimal==(int32_t)count*1000);
        CHECK(row.encCurrentN==100 && row.slipV2TotalMs==10);
        CHECK(fabsf(row.x)<1e-6F && fabsf(row.y-count*1000.0F/PULSE_MILLIMETER)<0.01F);
    }
    CHECK(count==100 && logSourceEof(&file) && !logSourceError(&file));
    CHECK(logSourceClose(&file)==FR_OK && logSourceMetadata(&file)==NULL);
    CHECK(readLogDistance(1)>0);
    static AnalysisData distance[OPT_BUFF_SIZE]; memcpy(distance,PPAD,sizeof(distance));
    int16_t baseCount=numPPADarry;
    CHECK(readLogDistance(1)==baseCount && !memcmp(distance,PPAD,sizeof(distance)));
    autoRunState.seriesActive=true;
    CHECK(routeBuildFromLog(1,0)>0); uint32_t geometry=pathComputeRouteGeometryCrc32();
    CHECK(routeBuildFromLog(1,0)>0 && geometry==pathComputeRouteGeometryCrc32());
    CHECK(routeBuildFromLog(1,1)>0);
    CHECK(readLogDistance(1)>0); optimalTrace=BOOST_DISTANCE; CHECK(saveRun(100,0)==2);
    CHECK(strstr(savedMetadata(2),"primaryLogNumber=1,") && strstr(savedMetadata(2),"slipSourceLogNumber=0,"));
    autoRunState.seriesActive=false;
    int16_t slipCount=readLogDistanceSlip(1,2,NULL); CHECK(slipCount>0);
    CHECK(analyzedSlipSourceNumber==2 && runMemory.distance.slip.validLongMs[0]==1000);
    CHECK(runMemory.distance.slip.validLatMs[0]==1000 && runMemory.distance.slip.risk[0]>0);
    static AnalysisData slip[OPT_BUFF_SIZE]; memcpy(slip,PPAD,sizeof(slip));
    CHECK(readLogDistanceSlip(1,2,NULL)==slipCount && !memcmp(slip,PPAD,sizeof(slip)));
    CHECK(saveRun(30,0)==3 && strstr(savedMetadata(3),"slipSourceLogNumber=2,"));
    CHECK(readLogDistanceSlip(1,3,NULL)<0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testMarkerIndependentXY
// 処理概要     補正距離の後退を生左右距離によるXY積分へ混入させない
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testMarkerIndependentXY(void)
{
    resetMock(); initLog(); modeLOG=true;
    appendRecordRaw(10,1000,1000,1001,0,0);
    appendRecordRaw(20,800,2000,2001,0,3);
    appendRecordRaw(30,1600,3000,3001,0,0);
    encRightMarker=1600; CHECK(endLog());
    FIL file; LogSourceRow row; CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_OK);
    for(unsigned i=0;i<3;i++) { CHECK(logSourceNext(&file,&row)); CHECK(fabsf(row.x)<1e-6F); CHECK(fabsf(row.y-((i+1)*1000.0F+0.5F)/PULSE_MILLIMETER)<0.001F); }
    CHECK(row.encTotalOptimal==1600 && logSourceClose(&file)==FR_OK);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testSavingFailures
// 処理概要     保存各段階の失敗・短い書込でtmpと以前のbinを保持する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testSavingFailures(void)
{
    const char *ops[]={"write","sync","truncate","rename","close"};
    for(unsigned i=0;i<6;i++) {
        resetMock(); saveRun(30,0); initLog(); modeLOG=true; appendRecord(10,1000,0,0);
        failureOperation=ops[i%5]; failureExtension=i%5==3?".bin":".tmp"; shortTransfer=i==5;
        CHECK(!endLog() && !lastRunSaved && !modeLOG && findFile("2.tmp") && findFile("1.bin"));
        CHECK(!autoRunCompleteRun(&autoRunState,&(AutoRunPlan){0},2,true,false));
        failureOperation=NULL; shortTransfer=false;
        CHECK(logPrepareBinaryStorage() && logHasIncompleteTemp() && findFile("2.tmp") && findFile("1.bin"));
    }
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testCorruptionAndReadFailures
// 処理概要     全領域CRC・実長・旧形式・I/O失敗とCSV非フォールバックを検証する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testCorruptionAndReadFailures(void)
{
    for(unsigned region=0;region<4;region++) {
        resetMock(); saveRun(30,0); MockFile *bin=findFile("1.bin"); LogBinaryHeader h;
        CHECK(logBinaryDecodeHeader(bin->data,bin->size,&h));
        size_t offset=region==0?40:region==1?512:region==2?512+(size_t)h.rows*h.recordBytes:bin->size-2;
        bin->data[offset]^=1; makeFile("1.csv"); FIL file;
        CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)!=FR_OK && findFile("1.bin") && findFile("1.csv"));
        if(region<1 || region==2) CHECK(!logPrepareBinaryStorage() && !logRecoveryIsReady());
    }
    resetMock(); makeFile("1.csv"); FIL file;
    CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_NO_FILE);
    CHECK(logSourceOpen(&file,0,LOG_SOURCE_PATH)==FR_INVALID_PARAMETER);
    CHECK(readLogDistance(1)<0 && routeBuildFromLog(1,0)<0);
    for(unsigned fault=0;fault<6;fault++) {
        resetMock(); saveRun(30,0);
        failureOperation=fault==0?"open":fault==1?"seek":"read";
        failureExtension=".bin"; shortTransfer=fault==3; failureSkip=fault>=4?fault-3:0;
        CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)!=FR_OK && findFile("1.bin"));
    }
    resetMock(); saveRun(30,0); CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_OK);
    failureOperation="close"; failureExtension=".bin";
    CHECK(logSourceClose(&file)!=FR_OK && findFile("1.bin"));
    resetMock(); saveRun(30,0); CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_OK);
    MockFile *bin=findFile("1.bin"); bin->data[512+LOG_SIZE*29]^=1;
    LogSourceRow row; while(logSourceNext(&file,&row)) {}
    CHECK(logSourceError(&file) && !logSourceEof(&file)); CHECK(logSourceClose(&file)==FR_OK);
    resetMock(); saveRun(30,0); bin=findFile("1.bin"); --bin->size;
    CHECK(!logPrepareBinaryStorage() && findFile("1.bin"));
    resetMock(); saveRun(30,0); bin=findFile("1.bin"); bin->data[7]=1;
    CHECK(!logPrepareBinaryStorage() && findFile("1.bin"));
    resetMock(); saveRun(30,0); logBufferWriter.flushPending=true;
    CHECK(!logPrepareBinaryStorage()); logBufferWriter.flushPending=false;
    CHECK(logPrepareBinaryStorage());
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testSlipRowBounds
// 処理概要     軸別有効時間とON時間の不正・飽和・非有限値を拒否する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testSlipRowBounds(void)
{
    LogSourceRow row={.ROC=3000,.targetSpeed=100,.slipV2TotalMs=10,
        .slipV2ValidMs=10,.slipV2LatValidMs=3,.slipV2OnLongMs=3,
        .slipV2OnLatMs=2,.slipV2LatIntervalReason=256};
    CHECK(logSourceValidateSlipRow(&row));
    row.slipV2OnLatMs=4; CHECK(!logSourceValidateSlipRow(&row)); row.slipV2OnLatMs=2;
    row.slipV2OnLongMs=11; CHECK(!logSourceValidateSlipRow(&row)); row.slipV2OnLongMs=3;
    row.slipV2TotalMs=UINT16_MAX; CHECK(!logSourceValidateSlipRow(&row)); row.slipV2TotalMs=10;
    row.ROC=NAN; CHECK(!logSourceValidateSlipRow(&row)); row.ROC=3000;
    row.slipV2IntervalReason=512; CHECK(!logSourceValidateSlipRow(&row));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 testSourceMetadataAndStartSnapshot
// 処理概要     開始時参照番号を固定しCRC正常でも不適格なSLIP/PATH元を拒否する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void testSourceMetadataAndStartSnapshot(void)
{
    const char *keys[]={"primaryLogNumber=1,","emcStop=0.00,","slipSourceLogNumber=0,","logExpectedRows=100,"};
    const unsigned change[]={17,8,20,16};
    for(unsigned test=0;test<4;test++) {
        resetMock(); saveRun(100,0); CHECK(readLogDistance(1)>0);
        optimalTrace=BOOST_DISTANCE; initLog(); modeLOG=true;
        analyzedNumber=99; analyzedSlipSourceNumber=88;
        for(unsigned i=0;i<100;i++) appendRecord((uint16_t)((i+1)*10),(i+1)*1000,0,0);
        encRightMarker=100000; CHECK(endLog());
        CHECK(strstr(savedMetadata(2),"primaryLogNumber=1,") && strstr(savedMetadata(2),"slipSourceLogNumber=0,"));
        MockFile *bin=findFile("2.bin"); LogBinaryHeader h;
        CHECK(logBinaryDecodeHeader(bin->data,bin->size,&h)); bin->data[bin->size]=0;
        uint8_t *meta=bin->data+512+(size_t)h.rows*h.recordBytes+h.schemaBytes;
        char *field=strstr((char *)meta,keys[test]); CHECK(field);
        field[change[test]]='9';
        h.metadataCrc=~logBinaryCrcUpdate(UINT32_MAX,meta,h.metadataBytes);
        logBinaryEncodeHeader(bin->data,&h);
        CHECK(readLogDistanceSlip(1,2,NULL)<0 && findFile("2.bin"));
    }
    resetMock(); saveRun(100,0); MockFile *bin=findFile("1.bin"); LogBinaryHeader h;
    CHECK(logBinaryDecodeHeader(bin->data,bin->size,&h)); bin->data[bin->size]=0;
    uint8_t *meta=bin->data+512+(size_t)h.rows*h.recordBytes+h.schemaBytes;
    char *field=strstr((char *)meta,"closureValid=1,"); CHECK(field); field[13]='0';
    h.metadataCrc=~logBinaryCrcUpdate(UINT32_MAX,meta,h.metadataBytes); logBinaryEncodeHeader(bin->data,&h);
    CHECK(routeBuildFromLog(1,0)<0 && findFile("1.bin"));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 exportFixture
// 処理概要     PC変換・整理前後の計画比較用に実保存binと型付き行を出力する
// 引数         folder: 今回の検証出力フォルダ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void exportFixture(const char *folder, bool curved)
{
    if(!folder) return;
    resetMock();
    if(curved) {
        initLog(); modeLOG=true;
        for(unsigned i=0;i<370;i++) appendRecord((uint16_t)((i+1)*10),(i+1)*1000,
            i<360?(float)(i+1):360.0F,0);
        encRightMarker=360000; CHECK(endLog() && logLastPrimaryRouteValid());
    } else saveRun(50,0);
    char path[512]; snprintf(path,sizeof(path),"%s/1.bin",folder);
    FILE *out=fopen(path,"wb"); CHECK(out); MockFile *bin=findFile("1.bin");
    CHECK(fwrite(bin->data,1,bin->size,out)==bin->size && fclose(out)==0);
    snprintf(path,sizeof(path),"%s/rows.csv",folder); out=fopen(path,"wb"); CHECK(out);
    CHECK(fputs("cntlog,encCurrentN,gyroVal_Z,courseMarker,encTotalOptimal,ROC,x,y\n",out)>=0);
    FIL file; LogSourceRow row; CHECK(logSourceOpen(&file,1,LOG_SOURCE_PATH)==FR_OK);
    while(logSourceNext(&file,&row)) CHECK(fprintf(out,"%ld,%ld,%.9g,%ld,%ld,%.9g,%.9g,%.9g\n",(long)row.cntlog,(long)row.encCurrentN,(double)row.gyroValZ,(long)row.courseMarker,(long)row.encTotalOptimal,(double)row.ROC,(double)row.x,(double)row.y)>0);
    CHECK(!logSourceError(&file) && logSourceClose(&file)==FR_OK && fclose(out)==0);
    snprintf(path,sizeof(path),"%s/plans.txt",folder); out=fopen(path,"wb"); CHECK(out);
    CHECK(readLogDistance(1)>0);
    for(int i=0;i<numPPADarry;i++) CHECK(fprintf(out,"D %d %d %a\n",i,PPAD[i].ROC,(double)PPAD[i].boostSpeed)>0);
    optimalTrace=BOOST_DISTANCE; saveRun(50,0); CHECK(readLogDistanceSlip(1,2,NULL)>0);
    for(int i=0;i<numPPADarry;i++) CHECK(fprintf(out,"S %d %d %a\n",i,PPAD[i].ROC,(double)PPAD[i].boostSpeed)>0);
    CHECK(routeBuildFromLog(1,0)>0); CHECK(fprintf(out,"P %u %lu\n",routeCount,(unsigned long)pathComputeRouteGeometryCrc32())>0);
    CHECK(routeBuildFromLog(1,1)>0); CHECK(fprintf(out,"C %u %lu\n",routeCount,(unsigned long)pathComputeRouteGeometryCrc32())>0);
    CHECK(fclose(out)==0);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     現行bin専用の保存・型付き解析と失敗時原本保持を検証する
// 引数         argv[1]: 任意の検証出力フォルダ
// 戻り値       成功時0
/////////////////////////////////////////////////////////////////////
int main(int argc,char **argv)
{
    testStorageAndSnapshots(); testPlansAndReader(); testMarkerIndependentXY();
    testSavingFailures(); testCorruptionAndReadFailures(); testSlipRowBounds();
    testSourceMetadataAndStartSnapshot();
    exportFixture(argc>1?argv[1]:NULL,argc>2);
    puts("PASS: binary storage, typed reader, CRC and I/O faults, snapshots, DISTANCE/SLIP/PATH, CSV rejection");
    return 0;
}
