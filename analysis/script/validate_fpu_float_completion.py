"""残りのfloat化: CSV境界値、保存丸め、ビルド診断、経路互換を検証する。"""
from pathlib import Path
import json, re, subprocess, tempfile, hashlib
from fpu_float_validation import ROOT, PROJECT, run, snapshots, function_body, host_flags
OUT = ROOT / 'analysis/fpu-float-completion'
OUT.mkdir(parents=True, exist_ok=True)
HARNESS = r'''
#include "courseLogCsv.h"
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
int main(void) {
 CourseLogColumnMap m; CourseLogDistanceRow d; CourseLogSlipRow s;
 assert(courseLogResolveHeader("ROC,encTotalOptimal,courseMarker,gyroVal_Z,encCurrentN,cntlog,targetSpeed,optimalIndex,slipFlag,slipFlagLat", "", true, &m));
 assert(courseLogParseDistanceRow("3.25,2147483647,-2147483648,-1.5,16777217,16777219,70.5,16777221,1,0",&m,&d));
 assert(d.encTotalOptimal==INT32_MAX && d.courseMarker==INT32_MIN && d.encCurrentN==16777217 && d.cntlog==16777219 && d.gyroValZ==-1.5f && d.ROC==3.25f);
 assert(courseLogParseSlipRow("3.25,2147483647,-2147483648,-1.5,16777217,16777219,70.5,16777221,1,0",&m,&s));
 assert(s.optimalIndex==16777221 && s.targetSpeed==70.5f && s.encTotalOptimal==INT32_MAX);
 const char *bad[]={"nan,1,0,1,1,1", "inf,1,0,1,1,1", "1e100,1,0,1,1,1", "1,2147483648,0,1,1,1", "1,-2147483649,0,1,1,1", "1,1,0,1,1", "1,,0,1,1,1", "1,1,0,1,1,1x"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(!courseLogParseDistanceRow(bad[i],&m,&d));
 assert(!courseLogParseSlipRow("1,1,0,1,1,1,70.5,1,1",&m,&s));
 // SDの有効範囲0..99を超えて、半整数の直前・ちょうど・直後を比較する。
 for(int i=-10000;i<=10000;i++) {
  float v=(float)i+0.5f;
  float p[]={nextafterf(v,-INFINITY),v,nextafterf(v,INFINITY)};
  for(int j=0;j<3;j++) assert((int32_t)round((double)p[j])==(int32_t)roundf(p[j]));
 }
 puts("CSV integer precision, invalid fields and SD rounding: PASS");
 return 0;
}
'''

UI_TEST = r'''
#include "setup.h"
#include "switch.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static struct {uint16_t cntSwitchUD,cntSwitchLR,cntSwitchUDLong,cntSwitchLRLong;} setupTimer;
int8_t pushUD,pushLR; uint8_t swValTact;
'''
UI_MAIN = r'''
static void tick(uint8_t key) {
 swValTact=key; setupTimer.cntSwitchUD=50;
}
int main(void) {
 float f=0.0f;
 tick(SW_UP); dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);assert(f==0.1f);
 tick(SW_UP); dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);assert(f==0.1f);
 tick(SW_NONE);dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);
 tick(SW_UP); dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);assert(f==0.2f);
 setupTimer.cntSwitchUDLong=PUSHTIME;
 tick(SW_UP); dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);assert(f==0.0f);
 tick(SW_DOWN); dataTuning(&f,0.1f,0.0f,0.2f,UD,TYPE_FLOAT);assert(f==0.2f);
 memset(&setupTimer,0,sizeof(setupTimer));pushUD=0;
 int16_t i=32766;
 tick(SW_UP);dataTuning(&i,1.0f,32760.0f,32767.0f,UD,TYPE_INT16);assert(i==32767);
 memset(&setupTimer,0,sizeof(setupTimer));pushUD=0;
 f=0.2f;
 tick(SW_UP);dataTuning(&f,0.1f,0.2f,0.0f,UD,TYPE_FLOAT);assert(f==0.1f);
 puts("UI short/long press, wrap, reversed limits and int16: PASS");
 return 0;
}
'''

with tempfile.TemporaryDirectory() as tmp:
 directory=Path(tmp)
 ui=directory/'ui.c';ui.write_text(UI_TEST+function_body((PROJECT/'Core/Src/setup.c').read_text(encoding='utf-8'), 'dataTuning')+UI_MAIN,encoding='utf-8')
 ui_exe=directory/'ui.exe'
 run(['gcc',*host_flags(),'-Wall','-Wextra','-Werror',ui,'-o',ui_exe])
 run([ui_exe],OUT/'ui-tests.txt')
 c=directory/'boundary.c';c.write_text(HARNESS,encoding='utf-8')
 exe=directory/'boundary.exe'
 run(['gcc','-std=c11','-Wall','-Wextra','-Werror',f'-I{PROJECT / "Core/Inc"}',c,PROJECT/'Core/Src/courseLogCsv.c','-lm','-o',exe])
 run([exe],OUT/'boundary-tests.txt')
 manifest=json.loads((ROOT/'analysis/fpu-float-audit/float/manifest.json').read_text(encoding='utf-8'))
 for primary in manifest['primary_logs']:
  p=Path(primary['path']); snapshots(directory,OUT,p)
  for suffix in ('route.csv','xy.json'):
   old=ROOT/'analysis/fpu-float-audit/float'/f'{p.stem}-{suffix}'
   new=OUT/f'{p.stem}-{suffix}'
   assert old.read_bytes()==new.read_bytes(), f'Primary comparison differs: {p.stem}-{suffix}'
run(['powershell','-NoProfile','-File',PROJECT/'tests/run_pc_tests.ps1'],OUT/'pc-tests.txt')
run(['powershell','-NoProfile','-File',PROJECT/'tests/run_log_xy_tests.ps1'],OUT/'xy-tests.txt')
run(['powershell','-NoProfile','-File',PROJECT/'tests/run_log_schema_tests.ps1'],OUT/'schema-tests.txt')
commands=json.loads((PROJECT/'build/Debug/compile_commands.json').read_text(encoding='utf-8'))
diagnostics=[]
for entry in commands:
 if '/Core/Src/' not in entry['file'].replace('\\','/'):continue
 tokens=entry['command'].split();compiler=tokens[0].replace('\\\\','\\')
 args=[t for t in tokens if t.startswith('-I') or (t.startswith('-D') and not t.startswith('-DGIT_'))]
 diagnostics.append(run([compiler,*args,'-mcpu=cortex-m4','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-std=gnu11','-fsyntax-only','-Wdouble-promotion','-Wfloat-conversion','-Wunsuffixed-float-constants',entry['file']]))
(OUT/'compiler-diagnostics.txt').write_text('\n'.join(diagnostics),encoding='utf-8')
summary={}
for config in ('Debug','Release'):
 build=PROJECT/f'build/{config}'
 run(['cmake','--build',build,'--clean-first'],OUT/f'after-{config}.txt')
 assembly=run(['arm-none-eabi-objdump','-d',build/'robotrace_v2.elf'],OUT/f'after-{config}.asm')
 calls={};function=''
 for line in assembly.splitlines():
  m=re.match(r'^[0-9a-f]+ <([^>]+)>:',line)
  if m:function=m[1]
  m=re.search(r'\bbl\S*\s+.*<(__aeabi_(?:d\w+|f2d))>',line)
  if m:calls.setdefault(function,[]).append(m[1])
 (OUT/f'after-{config}-double-calls.json').write_text(json.dumps(calls,indent=2),encoding='utf-8')
 before=(OUT/f'before-{config}.txt').read_text(encoding='utf-8-sig',errors='replace')
 after=(OUT/f'after-{config}.txt').read_text(encoding='utf-8-sig',errors='replace')
 # 行番号の移動を除外してwarning本文・種類の一致を確認する。
 warnings=lambda t:sorted(re.findall(r'warning: ([^\n\r]+)',t))
 assert warnings(before)==warnings(after),f'Warning set differs: {config}'
 summary[config]={'warnings':len(warnings(after)),'sha256':hashlib.sha256((build/'robotrace_v2.elf').read_bytes()).hexdigest()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print('Debug/Release, unchanged warnings, CSV boundaries, rounding, PC/XY/schema, primary replay: PASS')
