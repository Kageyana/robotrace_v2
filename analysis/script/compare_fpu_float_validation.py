"""Compare saved firmware audits; physical driving results are a separate acceptance gate."""
from pathlib import Path
import csv
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parents[2] / 'analysis/fpu-float-audit'


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def warnings_key(lines):
    return sorted(re.sub(r':\d+:\d+: warning:', ': warning:', s) for s in lines)


def main():
    baseline = read_json(ROOT / 'baseline/manifest.json')
    candidate = read_json(ROOT / 'float/manifest.json')
    report = ['# FPU単精度化のローカル比較結果', '',
              f'変更前: `{baseline["commit"]}`', f'float化: `{candidate["commit"]}`', '',
              '実機の処理時間・走行安定性は未確認。以下はビルド、静的診断、PCリプレイの結果。', '',
              '|構成|変更前の倍精度呼出し箇所|float化後|新規通常warning|',
              '|---|---:|---:|---:|']
    for config in ('Debug', 'Release'):
        calls = []
        for variant, manifest in (('baseline', baseline), ('float', candidate)):
            archive = ROOT / variant / config
            assert hashlib.sha256((archive / 'robotrace_v2.elf').read_bytes()).hexdigest() == manifest['builds'][config]['elf_sha256']
            calls.append(read_json(archive / 'double-calls.json'))
        old_warning = warnings_key(baseline['builds'][config]['normal_warnings'])
        new_warning = warnings_key(candidate['builds'][config]['normal_warnings'])
        added = [s for s in new_warning if s not in old_warning]
        assert not added, added
        report.append(f'|{config}|{sum(map(len, calls[0].values()))}|{sum(map(len, calls[1].values()))}|{len(added)}|')
        changes = {name: [len(calls[0].get(name, [])), len(calls[1].get(name, []))]
                   for name in sorted(set(calls[0]) | set(calls[1]))
                   if len(calls[0].get(name, [])) != len(calls[1].get(name, []))}
        (ROOT / f'{config}-double-call-changes.json').write_text(json.dumps(changes, indent=2), encoding='utf-8')
    report += ['', '## 同じ一次ログによる経路・XY比較', '',
               '経路比較は既存PCテストの固定速度設定を使用。XYは実装関数で累積距離と保存yawを再生。', '',
               '|ログ|経路整数値の差|ゴールX差[mm]|終了XY差[mm]|閉路X判定|',
               '|---|---:|---:|---:|---|']
    for old, new in zip(baseline['primary_logs'], candidate['primary_logs']):
        assert old['sha256'] == new['sha256'] and old['rows'] == new['rows']
        stem = Path(old['path']).stem
        old_route = list(csv.reader((ROOT / 'baseline' / f'{stem}-route.csv').open(encoding='utf-8')))
        new_route = list(csv.reader((ROOT / 'float' / f'{stem}-route.csv').open(encoding='utf-8')))
        assert old['route_results'] == new['route_results'], (stem, 'route acceptance changed')
        assert len(old_route) == len(new_route)
        differences = [{'line': i + 1, 'before': a, 'after': b}
                       for i, (a, b) in enumerate(zip(old_route, new_route)) if a != b]
        (ROOT / f'{stem}-route-differences.json').write_text(json.dumps(differences, indent=2), encoding='utf-8')
        ox, nx = old['xy'], new['xy']
        assert ox['goal_x_valid'] == nx['goal_x_valid'], (stem, 'closure X acceptance changed')
        delta_goal = nx['goal_x_mm'] - ox['goal_x_mm']
        delta_xy = ((nx['x_mm'] - ox['x_mm']) ** 2 + (nx['y_mm'] - ox['y_mm']) ** 2) ** .5
        report.append(f'|{stem}|{len(differences)}行|{delta_goal:.6f}|{delta_xy:.6f}|一致|')
        if differences:
            report.append(f'\n{stem}の整数差は`{stem}-route-differences.json`を確認し、原因説明前は実機比較へ進まない。\n')
    report += ['', '## 確認状況', '',
               '- 両版のDebug/Releaseビルド、既存PC経路・XY・ログスキーマテスト、計測集計テストは成功。',
               '- 制御経路の倍精度呼出し変化は構成別double-call-changes.jsonに保存。ELF全体には表示・CSV等のdouble処理が残る。',
               '- 計測対象はDWTリセット後から集計直前まで。ISR全体の実行時間ではない。',
               '- 実機での交互走行、DWT集計、10完了セット/版の比較、採用判断は未実施。']
    (ROOT / 'comparison.md').write_text('\n'.join(report) + '\n', encoding='utf-8')
    print('\n'.join(report))


if __name__ == '__main__':
    main()
