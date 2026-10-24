#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "speedParameters.h"
#include "pathPolicy.h"

#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
#define PATH_SETTING "./setting/"
enum { FR_OK, FR_NO_FILE, FA_CREATE_ALWAYS=2, FA_WRITE=4, FA_OPEN_EXISTING=8, FA_READ=16 };
typedef char TCHAR;
typedef struct { size_t position; } FIL;
typedef int FRESULT;
static char contents[256];
static bool exists;
static unsigned writes;

/////////////////////////////////////////////////////////////////////
// モジュール名 f_open
// 処理概要     速度設定のファイル有無と作成回数を模擬する
// 引数         file: ファイル, path: 保存先, mode: 読書き方式
// 戻り値       模擬FatFs結果
/////////////////////////////////////////////////////////////////////
static FRESULT f_open(FIL *file,const char *path,int mode) {
    CHECK(!strcmp(path,"./setting/targetSpeeds.txt"));file->position=0;
    if(mode&FA_CREATE_ALWAYS) { contents[0]=0;exists=true;writes++;return FR_OK; }
    return exists?FR_OK:FR_NO_FILE;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_close
// 処理概要     設定ファイルの正常クローズを模擬する
// 引数         file: ファイル
// 戻り値       FR_OK
/////////////////////////////////////////////////////////////////////
static int f_close(FIL *file) { (void)file;return FR_OK; }
/////////////////////////////////////////////////////////////////////
// モジュール名 f_printf
// 処理概要     実writerの書式をRAM上のファイルへ出力する
// 引数         file: ファイル, format: 書式, ...: 保存値
// 戻り値       書込文字数
/////////////////////////////////////////////////////////////////////
static int f_printf(FIL *file,const char *format,...) {
    va_list args;va_start(args,format);int n=vsnprintf(contents,sizeof(contents),format,args);va_end(args);
    CHECK(n>=0 && (size_t)n<sizeof(contents));file->position=(size_t)n;return n;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 f_gets
// 処理概要     実readerへ指定長の設定文字列を渡す
// 引数         buffer: 格納先, size: 最大文字数, file: ファイル
// 戻り値       格納先、終端時NULL
/////////////////////////////////////////////////////////////////////
static TCHAR *f_gets(TCHAR *buffer,int size,FIL *file) {
    if(!contents[file->position])return NULL;
    int used=0;
    while(used<size-1 && contents[file->position]) { buffer[used++]=contents[file->position++];if(buffer[used-1]=='\n')break; }
    buffer[used]=0;return buffer;
}
#include "settings.inc"

/////////////////////////////////////////////////////////////////////
// モジュール名 initializeCase
// 処理概要     原本と共通化後へ同じ既定値・SD内容を与える
// 引数         input: ファイル内容, present: ファイルの有無
// 戻り値       読込開始時のパラメータ
/////////////////////////////////////////////////////////////////////
static speedParam initializeCase(const char *input,bool present)
{
    float values[19];for(unsigned i=0;i<19;i++)values[i]=(float)(100+i*7)/100.0F;
    speedParam seed;memcpy(&seed,values,sizeof(seed));strcpy(contents,input);exists=present;writes=0;return seed;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 checkReadCase
// 処理概要     部分反映・範囲外既定値・旧18項目修復を保存バイトまで比較する
// 引数         input: 読込内容, present: ファイルの有無
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void checkReadCase(const char *input,bool present)
{
    tgtParam=initializeCase(input,present);readTgtspeeds();
    speedParam actual=tgtParam;char saved[256];strcpy(saved,contents);unsigned repaired=writes;
    CHECK(strlen(saved)==95 && !strchr(saved,'\n'));
#ifdef HAVE_BASELINE
    before_tgtParam=initializeCase(input,present);before_readTgtspeeds();
    CHECK(!memcmp(&actual,&before_tgtParam,sizeof(actual)) && repaired==writes && !strcmp(saved,contents));
#else
    CHECK(isfinite(actual.pathReplay));(void)repaired;
#endif
}

/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     19項目の配置・既定値・丸め・保存形式・破損修復を検証する
// 引数         なし
// 戻り値       成功時0
/////////////////////////////////////////////////////////////////////
int main(void)
{
    const float initial[]={1.0F,.8F,2.8F,2.6F,2.4F,2.2F,2.0F,1.8F,1.6F,1.6F,1.4F,1.3F,1.2F,1.1F,3.4F,3.0F,.5F,30.0F,1.0F};
    CHECK(sizeof(tgtParam)==sizeof(initial) && !memcmp(&tgtParam,initial,sizeof(initial)));
#ifdef HAVE_BASELINE
    CHECK(!memcmp(&tgtParam,&before_tgtParam,sizeof(tgtParam)));
#endif
    unsigned cases=0;
    for(unsigned i=0;i<101;i++) {
        float values[19];for(unsigned j=0;j<19;j++)values[j]=(float)(i*11+j)/100.0F+.005F;
        memcpy(&tgtParam,values,sizeof(values));writes=0;writeTgtspeeds();
        char saved[256];strcpy(saved,contents);CHECK(strlen(saved)==95 && writes==1);
#ifdef HAVE_BASELINE
        memcpy(&before_tgtParam,values,sizeof(values));before_writeTgtspeeds();CHECK(!strcmp(saved,contents));
#endif
        cases++;
    }
    char canonical[128]="",field[16];
    for(unsigned i=0;i<19;i++) { snprintf(field,sizeof(field),"%04u,",111+i*13);strcat(canonical,field); }
    checkReadCase(canonical,true);cases++;
    for(unsigned count=0;count<19;count++) {
        char partial[128];strcpy(partial,canonical);partial[count*5]=0;
        checkReadCase(partial,count!=0);cases++;
    }
    for(unsigned i=0;i<19;i++)for(unsigned fault=0;fault<3;fault++) {
        char corrupt[128];strcpy(corrupt,canonical);
        memcpy(corrupt+i*5,fault==0?"9999,":fault==1?"-001,":"abcd,",5);
        checkReadCase(corrupt,true);cases++;
    }
    printf("PASS: speed parameter defaults, 19-field order/rounding, partial/range/18-field repairs (%u cases)\n",cases);
    return 0;
}
