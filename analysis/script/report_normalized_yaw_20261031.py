"""解析済み結果を検査し、角速度正規化の比較資料を生成する。"""
import json
from pathlib import Path

import numpy as np

from analyze_normalized_yaw_20261031 import (
    OUT, ROOT, diagnostics, load, plan_radii, save_csv, save_json, sha, snapshot_candidates,
    stats, straight_mask, segments,
)

VIS = Path('C:/Users/ucawa/.codex/visualizations/2026/11/02/01a19c81-ea32-77d2-b235-88c8c166eb65')


def main():
    summary = json.loads((OUT/'summary.json').read_text(encoding='utf-8'))
    assert summary['selection']['selected_distance'] == [13935,13936,13938,13939,13940,13941,
        14025,14027,14029,14032,14034,14036,14037,14074]
    assert not summary['rejected']
    assert len(summary['logs'])==14 and len(summary['primaries'])==9
    normal = [int(n) for n,x in summary['logs'].items() if x['normal_log_end']]
    failed = [int(n) for n,x in summary['logs'].items() if not x['normal_log_end']]
    assert len(normal)==10 and failed==[13936,13940,14025,14037]
    identities=[x['identity'] for x in summary['logs'].values()]+[x['identity'] for x in summary['primaries'].values()]
    assert all(sha(Path(x['path']))==x['sha256'] for x in identities)
    diagnostic_logs=[int(n) for n,x in summary['logs'].items() if 'gains' in x['diagnostics']]
    assert diagnostic_logs==[14034,14036,14037,14074]

    # 報告の割合は実保存P2/D0の高速直線だけ。元の全高速直線と分母を混同しない。
    candidate_rows=[]
    for number in diagnostic_logs:
        rr=[r for r in summary['candidates'] if r['log']==number and r['scope']=='fast_straight']
        base=next(r for r in rr if r['candidate']=='baseline')
        for r in rr:
            candidate_rows.append(dict(**r,delta_rms_change_percent=(r['delta_rms_p']/base['delta_rms_p']-1)*100,
                clip_change_percentage_points=(r['wheel_clip_fraction']-base['wheel_clip_fraction'])*100))
    save_csv(OUT/'candidate_comparison.csv',candidate_rows)

    # 直線境界を広く/狭くしても高目標角速度・実gyro振動が残ることを別に保全する。
    checks=dict(original_sha_count=len(identities),csv_logs=14,reference_primaries=9,
        normal_log_end_count=10,fault_log_count=4,diagnostic_logs=diagnostic_logs,
        input_crc_revalidated=False,firmware_changed=False,closed_loop_trial=False)
    comparisons=[]
    chart={}
    samples=[]
    for number in diagnostic_logs:
        meta,d,identity=load(number)
        _,pd,_=load(int(float(meta['primaryLogNumber'])))
        radii=plan_radii(pd)
        mask=straight_mask(d,radii)&(d['speed']>=3)&(d['lineKpUsed']==2)&(d['lineKdUsed']==0)
        ii=max(segments(d,mask),key=lambda a:d['cntlog'][a[-1]]-d['cntlog'][a[0]])
        selected=np.zeros(len(mask),dtype=bool);selected[ii]=True
        # 波形を削らず全保存点を埋め込む。候補値は実gyroを固定した瞬時P項のみ。
        rho=np.minimum(1.,2./np.maximum(d['speed'],.5))
        gyro=np.trunc(d['gyroVal_Z'])
        base=np.clip(np.floor(2*(d['targetAngularvelo']-gyro)/32),-1000,1000)
        both=np.clip(np.floor(2*rho*(d['targetAngularvelo']-gyro)/32),-1000,1000)
        target=np.clip(np.floor(2*(rho*d['targetAngularvelo']-gyro)/32),-1000,1000)
        assert np.array_equal(base[ii],d['lineTraceCtrl'][ii])
        chart[str(number)]=dict(start_mm=float(d['distance'][ii[0]]),end_mm=float(d['distance'][ii[-1]]),
            saved_rows=len(ii),normal_log_end=number in normal,
            values=[[float(d['cntlog'][i]-d['cntlog'][ii[0]]),round(float(d['targetAngularvelo'][i]),3),
                round(float(d['gyroVal_Z'][i]),3),float(base[i]),float(both[i]),float(target[i]),
                round(float(d['speed'][i]),3)] for i in ii])
        save_csv(OUT/f'{number}_chart_signal.csv',[dict(ms=v[0],target_yaw_dps=v[1],actual_yaw_dps=v[2],
            actual_delta_p=v[3],both_scaled_delta_p=v[4],target_scaled_delta_p=v[5],speed_mps=v[6])
            for v in chart[str(number)]['values']])
        comparisons.append(dict(log=number,**stats(d,selected)))
        # 図に載せる前に原JSONの計算結果と独立に主要数値を照合する。
        rerun=snapshot_candidates(d,straight_mask(d,radii)&(d['speed']>=3))
        for r in rerun:
            old=next(x for x in candidate_rows if x['log']==number and x['candidate']==r['candidate'])
            assert abs(r['delta_rms_p']-old['delta_rms_p'])<1e-12
            assert abs(r['wheel_clip_fraction']-old['wheel_clip_fraction'])<1e-12
        checks[str(number)]=dict(baseline_saved_points_match=len(ii),selected_waveform_statistics=stats(d,selected))
        if number in (14036,14074):
            # 個別点もCSVへ保存。時間波形と集計の間で符号や倍率を追跡できる。
            for i in ii[::max(1,len(ii)//10)]:
                samples.append(dict(log=number,cntlog=float(d['cntlog'][i]),speed_mps=float(d['speed'][i]),
                    targetAngularvelo=float(d['targetAngularvelo'][i]),gyroVal_Z=float(d['gyroVal_Z'][i]),
                    rho=float(rho[i]),baseline=float(base[i]),both_scaled=float(both[i]),target_scaled=float(target[i])))
    save_json(OUT/'chart_data.json',chart)
    save_csv(OUT/'sample_candidates.csv',samples)
    save_csv(OUT/'chart_window_metrics.csv',comparisons)
    VIS.mkdir(exist_ok=True,parents=True)
    save_json(VIS/'normalized-yaw-data.json',chart)
    template=(ROOT/'analysis/script/normalized_yaw_chart.html').read_text(encoding='utf-8')
    assert template.count('__REVIEWED_CHART_DATA__')==1
    fragment=template.replace('__REVIEWED_CHART_DATA__',json.dumps(chart,ensure_ascii=False,separators=(',',':')))
    assert len(fragment.encode('utf-8'))<1_000_000
    (VIS/'normalized-yaw-feedback.html').write_text(fragment,encoding='utf-8')

    c36={r['candidate']:r for r in candidate_rows if r['log']==14036}
    lines=['# 10/31以降のDISTANCE：角速度正規化の検証', '',
        '作業ブランチ: `codex/normalized-yaw-feedback`。ファームウェア・SD設定・実機への変更は行っていない。', '',
        '## 判断', '',
        '速度に応じて操舵指令を弱める案には検証する価値がある。ただし、目標と実測の単純な正規化が発振を解消するとは実ログから証明できない。高速直線では過大な目標振動、追従の位相差、左右輪目標の制限が共存する。目標生成側の速度依存を抑える案を、ジャイロFBの強さを保った候補として先に試す根拠が得られた。これは実機採用判断ではない。', '',
        '## 対象と品質', '',
        '- 選定: CSVのLastWriteTimeを日本時間で2026-10-31 00:00以降に限定し、optimalTrace=2かつslipSourceLogNumber=0をDISTANCEとした。ファイル保存日時は実走日時の完全な保証ではなく、buildDateは選定へ使っていない。',
        '- 対象14本、参照一次9本。全23CSVのSHA256、全行幅、有限値、期待行数、cntlogの増加、58019 pulse/mを確認。除外・読込失敗0本。原binはこのフォルダに存在せず、全CRC再検証は未実施。',
        '- 正常ログ終了10本、emcStop=5の停止4本（13936/13940/14025/14037）。正常ログ終了を板内完走確認済みとは扱わない。停止ログは各表で明示し、正常10本へ加えない。',
        '- 実使用ゲインと左右輪目標を保存しているのは14034/14036/14037/14074の4本。その他10本の実ゲイン・輪制限率を終了ヘッダから確定しない。',
        '- 対象期間内にSLIPログは0本。本結果をSLIPの実測検証済みとは扱わない。', '',
        '## 方法', '',
        '各二次走行のprimaryLogNumberで一次を特定し、一次の50mmごとのROC中央値を現行readLogDistanceProgressと同じ境界順で参考再構成した。二次optimalIndexの前後2区間が全てR>=2800mmの点を直線内部とし、そのうち実速度3m/s以上を高速直線とした。二次gyroの小ささで選別していない。一次形状は自己推定で、物理的な床上位置の保証ではない。', '',
        '診断付き4本で再構成一次の先読みROCと保存先読みROCの一致率は97.7～99.7%。使用周期と保存時optimalIndexの差やCSV丸めがあるので全件の実計画再生とは呼ばない。前後1/2/4区間の境界感度をsensitivity.csvへ保存し、主要な高目標振動・実gyro振動・輪制限が残ることを確認した。', '',
        'RMSと率は次の保存点までの保持による時間重み付き近似。隣接2点とも同じ選別条件を満たし、間隔20ms以内の区間のみ使用する。記録は距離間隔で、未記録の1ms履歴を復元したものではない。', '',
        '## 全14本：一次形状で選んだ高速直線', '',
        '|ログ|終了|一次|対象秒|平均速度 m/s|目標角速度RMS deg/s|実角速度RMS deg/s|輪目標制限率|',
        '|---|---|---|---:|---:|---:|---:|---:|']
    for n,item in summary['logs'].items():
        s=item['fast_straight']
        clip=f"{s['wheel_clip_fraction']*100:.1f}%" if 'wheel_clip_fraction' in s else '未記録'
        lines.append(f"|{n}|{'正常ログ終了' if item['normal_log_end'] else '停止5'}|{item['primary']}|{s['seconds']:.3f}|{s['speed_mean']:.2f}|{s['target_yaw_rms']:.1f}|{s['gyro_rms']:.1f}|{clip}|")
    lines += ['', '速度設定、参照一次、ゲイン、電圧、制限値が期間内で変わっているため、ログ番号間のRMS差を正規化や単一変更の効果とは解釈しない。14074はbstStraight=3.7m/s、14036は4.2m/sであり、直線の位置も異なる。', '',
        '## 発振と乖離の実態', '',
        '- 診断付き4本の高速直線ではセンサー合成差×encCurrentN/512と目標角速度が全対象時間で1deg/s以内に一致。高速直線の目標値増大は現在の速度乗算式に整合する。低速小R向けFFや目標上限制限の影響を、この高速区間の説明に混入しない。',
        '- 14036/14037/14074の高速直線は実記録でP2/D0が100%。14034は約92%がP2/D0、残りがカーブ用P5/D12。計画ROCだけで切り替える版2でも振動は残っており、直線内のゲイン切替だけでは説明できない。',
        '- 14036の最初の一次形状直線内部（約4.66～8.15m）では目標/実gyro/ライン操舵/輪速度差の卓越周波数が約10.75Hz。目標→gyroの相互相関は+24msで最大（r=0.951）、輪目標差→実輪差は+6msで最大（r=0.935）。14074の長い直線でも約13.43Hz、目標→gyroは+21ms（r=0.961）、輪目標差→実輪差は+6ms（r=0.933）。',
        '- 上記は疎な保存点を1ms格子へ線形補間した閉ループ波形の時間対応診断。約20msをIMU単体の遅延、約6msをモーター単体の純遅延と断定しない。周期波形の別ピークや位相折返しもある。',
        '- 目標と実測の同時刻相関は14036で-0.171、14074で-0.281。大きさを揃えるだけでなく、振動と位相の検討が必要。両者を共通固定値で割る操作はゲインの変更に相当し、遅れを補償するものではない。', '',
        '## 速度正規化の固定入力比較', '',
        '比較用に rho=min(1, 2/max(v,0.5))（vはm/s）を使用した。2m/sは比較用の任意基準で、最適化・実機採用した設定ではない。2m/s以下では変更しない例である。', '',
        '```text',
        '現行:             delta = floor(2*(target - trunc(gyro))/32)',
        '目標・実測を共通縮小: delta = floor(2*rho*(target - trunc(gyro))/32)',
        '目標側だけ縮小:     delta = floor(2*(rho*target - trunc(gyro))/32)',
        '```', '',
        'deltaは片輪へ加算/反対輪から減算する速度差指令[pulse/ms]。実記録P2/D0・非マーカー点・高速直線に限定し、現行deltaとの一致を確認。両輪非飽和または片輪の自由側から使用した平均輪目標を復元し、±290pulse/msで候補制限率を計算した。Dが有効な周期は、未保存1ms差分が必要なので候補計算へ含めていない。', '',
        '|ログ|方式|対象秒|差分指令RMS pulse/ms|現行比|輪目標制限率|ジャイロP項RMS pulse/ms|',
        '|---|---|---:|---:|---:|---:|---:|']
    names=dict(baseline='現行',both_speed_scaled='両方を縮小',target_speed_scaled='目標のみ縮小')
    for r in candidate_rows:
        lines.append(f"|{r['log']}{'（停止）' if r['log']==14037 else ''}|{names[r['candidate']]}|{r['seconds']:.3f}|{r['delta_rms_p']:.2f}|{r['delta_rms_change_percent']:+.1f}%|{r['wheel_clip_fraction']*100:.1f}%|{r['p_gyro_component_rms_p']:.2f}|")
    lines += ['',
        f"14036では共通縮小で差分指令RMSが{c36['baseline']['delta_rms_p']:.2f}→{c36['both_speed_scaled']['delta_rms_p']:.2f}pulse/ms、候補輪制限率が{c36['baseline']['wheel_clip_fraction']*100:.1f}→{c36['both_speed_scaled']['wheel_clip_fraction']*100:.1f}%へ低下する。一方、ジャイロからのP項RMSも{c36['baseline']['p_gyro_component_rms_p']:.2f}→{c36['both_speed_scaled']['p_gyro_component_rms_p']:.2f}pulse/msへ弱まる。目標側だけの縮小では差分指令RMS{c36['target_speed_scaled']['delta_rms_p']:.2f}pulse/ms、候補輪制限率{c36['target_speed_scaled']['wheel_clip_fraction']*100:.1f}%、ジャイロP項{c36['target_speed_scaled']['p_gyro_component_rms_p']:.2f}pulse/msを保持する。", '',
        '**指令が小さくなることは演算上確認できたが、実gyroや蛇行が減ることは確認していない。** 表の候補制限率は元走行の速度・gyro・ライン偏差を固定した指令感度である。変更後は軌跡・センサー偏差・速度・フラグが変わるため、そのまま実走の改善率へ変換できない。', '',
        '## 次の実装・実機確認の方針', '',
        '1. 共通固定値による正規化を発振対策として採用しない。倍率調整として扱う。',
        '2. まず直線P2/D0区間の目標生成だけに速度依存の上限/緩和を設け、gyro FB、カーブ制御、輪速度PID、緊急停止を同時に変えない案を試す。速度正規化の全速度域・全モードへの一括適用は未検証。',
        '3. 同じ一次・速度設定で変更前後を比較し、目標とgyroの振動、輪制限、ライン偏差、完走・ラップを評価する。最初の1走でカーブ移行と停止条件を確認後、同条件10本以上で再現性を判定する。',
        '4. SLIPは今回の対象にないので別に実走検証する。候補方式の順位・安全性・最適基準速度は今回の固定入力比較だけでは確定しない。', '',
        '## 再実行と成果物', '',
        '```powershell',
        "& 'C:/Users/ucawa/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' analysis/script/analyze_normalized_yaw_20261031.py",
        "& 'C:/Users/ucawa/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' analysis/script/report_normalized_yaw_20261031.py",
        '```', '',
        'summary.jsonに全メタデータ・原本SHA・全区間・相互相関候補を保存。inventory.csvは対象期間内全CSVの分類、bands.csvは速度帯、sensitivity.csvは直線境界の感度、candidate_comparison.csvは固定入力候補、各chart_signal.csvは図の全保存点。validation.jsonは再計算と原本不変照合の結果。', '']
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf-8')
    checks['analysis_source_sha256']=sha(ROOT/'analysis/script/analyze_normalized_yaw_20261031.py')
    checks['report_source_sha256']=sha(Path(__file__))
    save_json(OUT/'validation.json',checks)
    print('PASS: 14 DISTANCE + 9 primary SHA; 4 diagnostic logs; baseline and candidates independently rechecked')
    print('CHART rows', {k:v['saved_rows'] for k,v in chart.items()})
    print('CANDIDATES', [{k:r[k] for k in ['log','candidate','delta_rms_p','wheel_clip_fraction','p_gyro_component_rms_p']} for r in candidate_rows])


if __name__=='__main__':
    main()
