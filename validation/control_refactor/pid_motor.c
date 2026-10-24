#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
#include "types.inc"
typedef int FontDef;
enum { Font_6x8=6, Font_7x10=7, SW_PUSH=1 };
static Pattern pattern;
static TestFlags testFlags;
static pidParam lineTraceCtrl, lineTraceOmegaFBCtrl, veloCtrl, veloCtrlL, veloCtrlR;
static int16_t speedFeedForwardGain, encCurrentL;
static char trace[8192];
static size_t traceSize;
static int toggle, editDelta;
static int16_t outL,outR;

/////////////////////////////////////////////////////////////////////
// モジュール名 event
// 処理概要     描画・出力・操作の呼出し順と値を記録する
// 引数         format: イベント書式, ...: 記録値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void event(const char *format,...)
{
    va_list args; va_start(args,format);
    int n=vsnprintf(trace+traceSize,sizeof(trace)-traceSize,format,args); va_end(args);
    CHECK(n>=0 && (size_t)n<sizeof(trace)-traceSize); traceSize+=(size_t)n;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 motorCommandOut
// 処理概要     左右モーター指令を保持し呼出し順を記録する
// 引数         l: 左指令, r: 右指令
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void motorCommandOut(int16_t l,int16_t r) { outL=l;outR=r;event("M%d/%d;",l,r); }
/////////////////////////////////////////////////////////////////////
// モジュール名 powerLineSensors
// 処理概要     センサー点灯要求を記録する
// 引数         value: 点灯状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void powerLineSensors(uint8_t value) { event("L%u;",value); }
/////////////////////////////////////////////////////////////////////
// モジュール名 setTargetSpeed
// 処理概要     目標速度設定の呼出しを記録する
// 引数         value: 目標速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void setTargetSpeed(float value) { event("V%.1f;",(double)value); }
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_SetCursor
// 処理概要     画面カーソル位置を記録する
// 引数         x: 横位置, y: 縦位置[pixel]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void ssd1306_SetCursor(uint8_t x,uint8_t y) { event("C%u/%u;",x,y); }
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_printf
// 処理概要     通常文字のフォントと描画内容を記録する
// 引数         font: フォント, format: 書式, ...: 表示値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void ssd1306_printf(FontDef font,char *format,...) {
    char buffer[64];va_list args;va_start(args,format);vsnprintf(buffer,sizeof(buffer),format,args);va_end(args);event("T%d:%s;",font,buffer);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 ssd1306_printfB
// 処理概要     選択文字のフォントと描画内容を記録する
// 引数         font: フォント, format: 書式, ...: 表示値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void ssd1306_printfB(FontDef font,char *format,...) {
    char buffer[64];va_list args;va_start(args,format);vsnprintf(buffer,sizeof(buffer),format,args);va_end(args);event("B%d:%s;",font,buffer);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 data_select
// 処理概要     選択ボタンの入力を模擬する
// 引数         value: 選択状態, button: 対象ボタン
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void data_select(uint8_t *value,uint8_t button) { event("S%u;",button);if(toggle)*value=(uint8_t)(!*value); }
/////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUD
// 処理概要     項目選択の引数を記録する
// 引数         value: 選択値, add: 調整幅, min/max: 選択範囲
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void dataTuningUD(int16_t *value,int16_t add,int16_t min,int16_t max) { (void)value;event("U%d/%d/%d;",add,min,max); }
/////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningLR
// 処理概要     ゲイン編集と範囲端の折返しを模擬する
// 引数         value: 編集値, add: 調整幅, min/max: 編集範囲
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void dataTuningLR(int16_t *value,int16_t add,int16_t min,int16_t max) {
    event("E%d/%d/%d;",add,min,max);int v=*value+editDelta;*value=(int16_t)(v<min?max:v>max?min:v);
}
#include "production.inc"

typedef struct {
    Pattern pattern;
    TestFlags flags;
    pidParam line,omega,speed,left,right;
    int16_t ff,enc;
} State;
/////////////////////////////////////////////////////////////////////
// モジュール名 capture
// 処理概要     比較対象のUI・ゲイン・試験状態を保持する
// 引数         なし
// 戻り値       現在の比較対象状態
/////////////////////////////////////////////////////////////////////
static State capture(void) {
    State s={0};s.pattern=pattern;s.flags=testFlags;s.line=lineTraceCtrl;s.omega=lineTraceOmegaFBCtrl;
    s.speed=veloCtrl;s.left=veloCtrlL;s.right=veloCtrlR;s.ff=speedFeedForwardGain;s.enc=encCurrentL;return s;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 restore
// 処理概要     同一状態から現行と変更前の画面処理を開始する
// 引数         s: 復元状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void restore(State s) {
    pattern=s.pattern;testFlags=s.flags;lineTraceCtrl=s.line;lineTraceOmegaFBCtrl=s.omega;veloCtrl=s.speed;
    veloCtrlL=s.left;veloCtrlR=s.right;speedFeedForwardGain=s.ff;encCurrentL=s.enc;traceSize=0;trace[0]=0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     飽和境界とPID全画面の描画・試験遷移・編集を検証する
// 引数         なし
// 戻り値       成功時0
/////////////////////////////////////////////////////////////////////
int main(void)
{
    unsigned motors=0,frames=0;
    const int16_t values[]={-1000,-999,-500,-1,0,1,500,999,1000};
    for(unsigned t=0;t<9;t++)for(unsigned s=0;s<9;s++)for(unsigned y=0;y<9;y++) {
        traceSize=0;motorCommandOutSynth(values[t],values[s],values[y]);
        CHECK(outL>=-1000 && outL<=1000 && outR>=-1000 && outR<=1000);
#ifdef HAVE_BASELINE
        int16_t l=outL,r=outR;traceSize=0;before_motorCommandOutSynth(values[t],values[s],values[y],0);CHECK(outL==l && outR==r);
#endif
        motors++;
    }
    State seed={0};seed.line=(pidParam){"line",14,0,60,0,123};seed.omega=(pidParam){"omega",4,0,1,0,-321};
    seed.speed=(pidParam){"speed",12,0,4,0,99};seed.ff=150;
    for(int mode=0;mode<3;mode++)for(int enter=0;enter<2;enter++)for(int state=0;state<3;state++)
    for(int before=0;before<3;before++)for(int enc=0;enc<2;enc++)for(int gain=1;gain<=4;gain++)
    for(editDelta=-1;editDelta<=1;editDelta++)for(toggle=0;toggle<2;toggle++) {
        State input=seed;input.pattern.display=5;input.pattern.beforeHex=(uint8_t)(enter?0:5);
        input.pattern.gain=(int16_t)gain;input.flags.trace_test=(uint8_t)state;input.flags.beforeMotorTest=(uint8_t)before;input.enc=(int16_t)(enc?7:0);
        restore(input);setup_pid(mode==0?&lineTraceCtrl:mode==1?&lineTraceOmegaFBCtrl:&veloCtrl,mode==2);
        State actual=capture();char events[8192];strcpy(events,trace);
        if(mode==2) CHECK(actual.left.kp==input.speed.kp && actual.right.kd==input.speed.kd);
        if(actual.flags.trace_test!=0) CHECK(actual.ff==input.ff);
#ifdef HAVE_BASELINE
        restore(input);
        if(mode==0)before_setup_pid_trace();else if(mode==1)before_setup_pid_traceOmegaFB();else before_setup_pid_speed();
        State expected=capture();CHECK(!strcmp(events,trace));CHECK(!memcmp(&actual,&expected,sizeof(actual)));
#else
        CHECK(events[0]!=0);
#endif
        frames++;
    }
    printf("PASS: motor saturation %u cases and PID UI %u frames (display, gains, motor outputs, transitions)\n",motors,frames);
    return 0;
}
