#include "logMetadata.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "pathPolicy.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)

typedef const char *FIL;
typedef struct { uint32_t expectedRows; bool closureValid; } RouteSourceInfo;
#define BOOST_NONE 0
/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceMetadata
// 処理概要     抽出した実PATH読込へ検証用メタデータを渡す
// 引数         file: 検証入力
// 戻り値       メタデータ文字列
/////////////////////////////////////////////////////////////////////
static const char *logSourceMetadata(FIL *file) { return *file; }
#include "path_metadata.inc"

/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     共通メタデータ読込の数値表記・境界・不正入力を検証する
// 引数         なし
// 戻り値       0: 全検証成功
/////////////////////////////////////////////////////////////////////
int main(void)
{
    const struct { const char *text; int32_t expected; } valid[] = {
        {"key=0,",0},{"key=0.00,",0},{"key=100.00,",100},
        {"x=9,key=-42,other=5,",-42},{"key=2147483647",INT32_MAX},
        {"key=-2147483648\r\n",INT32_MIN},{"key=1e2,",100},
        {"longkey=9,key=7\n",7},{"key=-0.00,",0}
    };
    for (size_t i=0;i<sizeof(valid)/sizeof(valid[0]);i++) {
        int32_t value=1234;
        CHECK(logMetadataReadInt(valid[i].text,"key",&value) && value==valid[i].expected);
    }
    const char *invalid[] = {"", "other=1,", "longkey=1,", "keyOther=1,", "key=,",
        "key=1junk,", "key=1.5,", "key=nan,", "key=inf,", "key=-inf,",
        "key=2147483648,", "key=-2147483649,", "key=1e999,", "key=1e-999,",
        "key=1,key=1,", "key=1,other=2,key=2,", "key=1 ,", "key=--1,"
    };
    for (size_t i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        int32_t value=1234;
        CHECK(!logMetadataReadInt(invalid[i],"key",&value) && value==1234);
    }
    int32_t value=1234;
    CHECK(!logMetadataReadInt(NULL,"key",&value));
    CHECK(!logMetadataReadInt("key=1,",NULL,&value));
    CHECK(!logMetadataReadInt("key=1,","key",NULL));
    const char *badKeys[]={"", "k,ey", "k=ey", "k\rey", "k\ney"};
    for (size_t i=0;i<sizeof(badKeys)/sizeof(badKeys[0]);i++)
        CHECK(!logMetadataReadInt("key=1,",badKeys[i],&value) && value==1234);
    FIL file="pathSourceFormatVersion=1,closureValid=1,closureReason=0,optimalTrace=0.00,"
        "emcStop=0.00,encoderPulsePerMeter=58019,imuCalibrationValid=1,"
        "imuCalibrationSamples=100,imuCalibrationReadErrors=0,distanceScaleVerified=1,logExpectedRows=100,";
    RouteSourceInfo source={0};
    CHECK(pathReadSourceMetadata(&file,&source) && source.expectedRows==100 && source.closureValid);
    puts("PASS: metadata numeric forms, int32 bounds, duplicates, exact keys and invalid input (35 cases)");
    return 0;
}
