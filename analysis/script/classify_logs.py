"""全ログ分類。入力CSVは読み取り専用。実移動はapply_classification.pyで行う。"""
from __future__ import annotations

import argparse
import collections
import csv
import difflib
import hashlib
import io
import json
import math
import re
from functools import lru_cache
from pathlib import Path

import numpy as np

DEFAULT_ROOT = Path(r"F:\Dropbox\Document\robotrace\Log\v2\old")
DEFAULT_OUT = Path(__file__).resolve().parents[1] / "output"
SEEDS = {"course_001": [12663], "course_002": [12378], "course_003": [12363],
         "course_004": [12282], "course_005": [12257, 12262]}
N = 512
VERSION = 3
WEIGHTS = {"markerPosition": .30, "markerSequence": .20, "turn": .20,
           "gyro": .15, "roc": .10, "distance": .05}
SELECTED = {"cntlog", "encTotalOptimal", "encTotalN", "gyroVal_Z", "encCurrentN",
            "ROC", "courseMarker", "markerSensor", "pathState", "sgMarkerCount"}


def number(value):
    """欠落・非有限・不正値はNone。欠落をゼロへ変換しない。"""
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def parse_csv(path):
    """cntlogを含む行を探し、列番号とメタデータを独立して読み取る。"""
    raw = path.read_bytes()
    # 一部旧ログのコメントはCP932。数値・列名はどちらでも同じ。
    try:
        content = raw.decode("utf-8-sig")
        encoding = "utf-8-sig"
    except UnicodeDecodeError:
        content = raw.decode("cp932", errors="replace")
        encoding = "cp932"
    nul_count = content.count("\0")
    content = content.replace("\0", "\ufffd")
    metadata, columns, arrays = {}, [], {}
    bad_rows = int(nul_count > 0)
    row_count = 0
    for row in csv.reader(io.StringIO(content, newline=None)):
        if not row or not any(v.strip() for v in row):
            continue
        if not columns:
            for cell in row:
                if "=" in cell:
                    key, val = cell.split("=", 1)
                    metadata[key.strip()] = val.strip()
            if "cntlog" not in [v.strip() for v in row]:
                continue
            # 同じ行末尾のメタデータ・空列はデータ列に含めない。
            columns = [v.strip() for v in row if v.strip() and "=" not in v]
            arrays = {name: [] for name in columns if name in SELECTED}
            continue
        row_count += 1
        values = row[:]
        while values and not values[-1].strip():
            values.pop()
        bad = len(values) != len(columns)
        for index, name in enumerate(columns):
            value = number(values[index]) if index < len(values) else None
            if value is None:
                bad = True
            if name in arrays:
                arrays[name].append(value)
        bad_rows += int(bad)
    return {"metadata": metadata, "columns": columns, "arrays": arrays,
            "rowCount": row_count, "badRows": bad_rows, "encoding": encoding,
            "nulBytes": nul_count, "sha256": hashlib.sha256(raw).hexdigest()}


def meta(log, key):
    return number(log["metadata"].get(key))


def array(log, key):
    values = log["arrays"].get(key)
    return np.asarray([np.nan if v is None else v for v in values]) if values else None


def completion(log):
    reasons, uncertain = [], []
    if not log["columns"] or not log["rowCount"]:
        reasons.append("missing columns or empty data")
    if log["badRows"]:
        reasons.append(f"malformed/missing numeric rows={log['badRows']}")
    emc = meta(log, "emcStop")
    if emc is not None and emc != 0:
        reasons.append(f"emcStop={emc:g}")
    for key in ("gyroSampleFault", "encoderIntervalFault", "logOverflowFinal", "dbgOverflowFinal"):
        value = meta(log, key)
        if value is not None and value != 0:
            reasons.append(f"{key}={value:g}")
    expected = meta(log, "logExpectedRows")
    if expected is not None and expected != log["rowCount"]:
        reasons.append("logExpectedRows mismatch")
    times = array(log, "cntlog")
    log["finalCntlog"] = float(times[-1]) if times is not None and np.isfinite(times[-1]) else None
    if times is not None and np.isfinite(times).all():
        delta = np.diff(times)
        # U16の折り返しだけを許容。固定時間間隔で欠落判定しない。
        wrapped = (delta < -32768) & (times[:-1] > 60000) & (times[1:] < 6000)
        if np.any((delta <= 0) & ~wrapped):
            reasons.append("cntlog nonmonotonic/duplicate")
        log["timeWraps"] = int(wrapped.sum())
    distance_key = "encTotalOptimal" if "encTotalOptimal" in log["columns"] else "encTotalN"
    distance = array(log, distance_key)
    log["distanceColumn"] = distance_key if distance is not None else ""
    log["finalDistancePulse"] = float(distance[-1]) if distance is not None and np.isfinite(distance[-1]) else None
    # 歴史的な換算値が不明なため分類はpulse比を使う。mmは現行換算で参考表示。
    log["finalDistance"] = log["finalDistancePulse"] / 58.019 if log["finalDistancePulse"] is not None else None
    if distance is None:
        uncertain.append("distance column missing")
    elif not np.isfinite(distance).all() or distance[-1] <= 0:
        reasons.append("distance invalid/nonfinite")
    else:
        decreases = np.diff(distance) < 0
        log["distanceBacksteps"] = int(decreases.sum())
        correction = float(np.max(np.maximum.accumulate(distance) - distance))
        log["distanceMaxCorrectionPulse"] = correction
        if decreases.any():
            if meta(log, "optimalTrace") == 0 or distance_key != "encTotalOptimal" or correction > distance[-1] * .10:
                reasons.append("distance invalid/nonmonotonic beyond allowed correction")
            else:
                # DISTANCEのマーカー再位置合わせは実際に記録された座標の戻り。
                # モード欠落は完走保留のまま、形状だけは保守的な単調包絡で比較。
                log["distanceAxis"] = "monotone envelope of corrected pulse"
    goal = meta(log, "goalMarkerOnset_p")
    if goal is not None and goal > 0 and log["finalDistancePulse"] is not None:
        if log["finalDistancePulse"] < goal * .99:
            reasons.append("final distance precedes recorded goal marker onset")
    mode = meta(log, "optimalTrace")
    path = mode in (3, 4)
    if path:
        states = array(log, "pathState")
        if states is not None and np.any(states == 4):
            reasons.append("pathState localization lost")
    else:
        sg = meta(log, "sgMarkerAtLogEnd")
        if sg is not None and sg < 2:
            uncertain.append("goal marker count < 2; historical goal count unknown")
        for key in ("startMarkerOnsetValid", "goalMarkerOnsetValid"):
            if meta(log, key) == 0:
                uncertain.append(f"{key}=0")
    reason = meta(log, "closureReason")
    # reason=8、PATH系reason=1だけでは異常としない。
    if meta(log, "closureValid") == 0 and reason != 8 and not (path and reason == 1):
        uncertain.append(f"closure invalid reason={reason}")
    if emc is None:
        uncertain.append("emcStop missing; completion cannot be proven")
    if reasons:
        return "ABNORMAL", reasons + uncertain
    if uncertain:
        return "PENDING", uncertain
    return "NORMAL_CANDIDATE", []


def features(log):
    distance = array(log, log["distanceColumn"])
    if distance is None or len(distance) < 10 or not np.isfinite(distance).all():
        return None
    if distance[-1] <= distance[0]:
        return None
    if np.any(np.diff(distance) < 0):
        correction = float(np.max(np.maximum.accumulate(distance) - distance))
        if meta(log, "optimalTrace") == 0 or log["distanceColumn"] != "encTotalOptimal" or correction > distance[-1] * .10:
            return None
        distance = np.maximum.accumulate(distance)
    # スタートマーカー原点からの距離を使用。逆向き比較は1-s。
    normalized = distance / distance[-1]
    keep = np.r_[True, np.diff(normalized) > 0]
    grid = np.linspace(0, 1, N)
    def resample(values):
        if values is None:
            return None
        good = keep & np.isfinite(values)
        if good.sum() < .95 * len(values):
            return None
        return np.interp(grid, normalized[good], values[good]).tolist()
    gyro = array(log, "gyroVal_Z")
    speed = array(log, "encCurrentN")
    # 速度変更で角速度振幅が変わるため、距離当たりの回転を形状比較に使う。
    gyro_shape = None
    if gyro is not None and speed is not None:
        gyro_shape = resample(gyro / np.where(np.abs(speed) > 1, np.abs(speed), np.nan))
    elif gyro is not None:
        gyro_shape = resample(gyro)
    roc = array(log, "ROC")
    roc_shape = None
    if roc is not None:
        curvature = 1 / np.clip(np.abs(roc), 80, 100000)
        if gyro is not None:
            curvature *= np.sign(gyro)
        roc_shape = resample(curvature)
    marker_key = "courseMarker" if "courseMarker" in log["columns"] else "markerSensor"
    markers = array(log, marker_key)
    events = None
    if markers is not None:
        events, previous = [], 0
        for position, value in zip(normalized, markers):
            if not math.isfinite(value):
                continue
            value = int(value)
            if value and value != previous:
                # 同一種類のチャタリングを0.3%距離以内でまとめる。
                if not events or events[-1][1] != value or position - events[-1][0] > .003:
                    events.append([float(position), value])
            previous = value
    turn = None
    turns = []
    if gyro_shape is not None:
        g = np.convolve(np.asarray(gyro_shape), np.ones(7) / 7, mode="same")
        threshold = max(float(np.percentile(np.abs(g), 80)) * .18, .05)
        turn = np.where(np.abs(g) >= threshold, np.sign(g), 0).tolist()
        start = 0
        for end in range(1, N + 1):
            if end == N or turn[end] != turn[start]:
                if turn[start] and end - start >= 3:
                    turns.append([turn[start], start / (N - 1), (end - start) / N])
                start = end
    return {"gyro": gyro_shape, "rawGyro": resample(gyro), "roc": roc_shape,
            "turn": turn, "turns": turns, "events": events,
            "markerColumn": marker_key if markers is not None else None,
            "distance": float(distance[-1]),
            "maxAngularVelocity": float(np.nanmax(np.abs(gyro))) if gyro is not None and np.isfinite(gyro).any() else None}


def reverse(feature, swap=False):
    result = {k: v for k, v in feature.items() if not k.startswith("_")}
    for key in ("gyro", "rawGyro", "roc", "turn"):
        if feature[key] is not None:
            result[key] = (-np.asarray(feature[key])[::-1]).tolist()
    if feature["events"] is not None:
        # SG右マーカーは進行方向を変えても右側。L/R交換は独立仮説として記録。
        result["events"] = [[1 - p, 3 - v if swap and v in (1, 2) else v]
                            for p, v in reversed(feature["events"])]
    return result


@lru_cache(maxsize=16384)
def marker_alignment(a, b):
    if a == b:
        return 1.0, [(0, 0, len(a))]
    matcher = difflib.SequenceMatcher(None, a, b, autojunk=False)
    return matcher.ratio(), [(v.a, v.b, v.size) for v in matcher.get_matching_blocks() if v.size]


def shape_correlation(a, b, key):
    """繰り返す代表比較の中心化・正規化を一度だけ計算する。"""
    if a[key] is None or b[key] is None:
        return None
    units = []
    for f in (a, b):
        cache = f.setdefault("_units", {})
        if key not in cache:
            values = np.asarray(f[key])
            centered = values - values.mean()
            norm = np.linalg.norm(centered)
            cache[key] = centered / norm if norm > 1e-12 else None
        units.append(cache[key])
    if any(v is None for v in units):
        return 1.0 if np.allclose(a[key], b[key]) else 0.0
    return float(np.clip(np.dot(units[0], units[1]), 0, 1))


def pair_score(a, b):
    scores = {"gyro": shape_correlation(a, b, "gyro"),
              "roc": shape_correlation(a, b, "roc"),
              "turn": None, "markerSequence": None, "markerPosition": None,
              "distance": min(a["distance"], b["distance"]) / max(a["distance"], b["distance"])}
    if a["turn"] is not None and b["turn"] is not None:
        if "_turn" not in a:
            a["_turn"] = np.asarray(a["turn"])
        if "_turn" not in b:
            b["_turn"] = np.asarray(b["turn"])
        at, bt = a["_turn"], b["_turn"]
        active = (at != 0) | (bt != 0)
        scores["turn"] = float(np.mean(at[active] == bt[active])) if active.any() else 1.0
    ea, eb = a["events"], b["events"]
    # raw markerSensorと確認済courseMarkerは意味が異なるので混ぜない。
    if ea is not None and eb is not None and a["markerColumn"] == b["markerColumn"]:
        la, lb = len(ea), len(eb)
        if la and lb:
            # 順序を保つ部分系列一致。一個欠落でも残りのイベントを照合する。
            sa, sb = tuple(v for _, v in ea), tuple(v for _, v in eb)
            sequence, blocks = marker_alignment(sa, sb)
            scores["markerSequence"] = sequence
            # 一個欠落時も、順序が対応する残りのマーカー位置を比較する。
            # AUTOの完全系列一致条件は維持する。
            if sequence >= .70:
                errors = [abs(ea[i+k][0] - eb[j+k][0]) for i, j, size in blocks for k in range(size)]
                error = float(np.mean(errors))
                scores["markerPosition"] = math.exp(-error / .035) * len(errors) / max(la, lb)
            else:
                scores["markerPosition"] = 0.0
        elif not la and not lb:
            # イベントが双方0個でも一致証拠にはしない。
            scores["markerSequence"] = scores["markerPosition"] = None
        else:
            scores["markerSequence"] = scores["markerPosition"] = 0.0
    available = sum(WEIGHTS[k] for k, value in scores.items() if value is not None)
    total = sum(WEIGHTS[k] * value for k, value in scores.items() if value is not None)
    return total / available if available else 0, scores


def compare(a, b):
    variants = [("same", a), ("reverse", reverse(a)), ("reverse_swap", reverse(a, True))]
    return compare_variants(variants, b)


def compare_variants(variants, b):
    return max(((score, direction, parts) for direction, f in variants
                for score, parts in [pair_score(f, b)]), key=lambda v: v[0])


def write_csv(path, rows, fields=None):
    fields = fields or (list(rows[0]) if rows else [])
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def write_cache(path, root, entries):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"version": VERSION, "root": str(root), "entries": entries},
                                   separators=(",", ":")), encoding="utf-8")
    temporary.replace(path)


def load_logs(root, out, refresh=False):
    cache_path = out / "classification_cache.json"
    cached = json.loads(cache_path.read_text(encoding="utf-8")) if cache_path.exists() and not refresh else {}
    if cached.get("version") != VERSION or cached.get("root") != str(root):
        cached = {}
    entries, inventory, logs = {}, [], []
    dirty_cache = False
    paths = sorted(root.glob("*.csv"))
    for directory in sorted(root.glob("course_[0-9][0-9][0-9]")):
        paths.extend(sorted(directory.glob("*.csv")))
    for index, path in enumerate(paths):
        relative = path.relative_to(root).as_posix()
        stat = path.stat()
        stamp = [stat.st_size, stat.st_mtime_ns]
        old = cached.get("entries", {}).get(relative)
        if old and old["stamp"] == stamp:
            log = old["log"]
        else:
            dirty_cache = True
            try:
                log = parse_csv(path)
            except (csv.Error, OSError) as error:
                log = {"metadata": {}, "columns": [], "arrays": {}, "rowCount": 0,
                       "badRows": 1, "encoding": "", "sha256": "", "parseError": str(error)}
            log["completionStatus"], log["completionReasons"] = completion(log)
            if log.get("parseError"):
                log["completionReasons"].append(log["parseError"])
            log["feature"] = features(log)
            log.pop("arrays")
        goal = meta(log, "goalMarkerOnset_p")
        final = log.get("finalDistancePulse")
        if goal is not None and goal > 0 and final is not None and final < goal * .99:
            log["completionStatus"] = "ABNORMAL"
            reason = "final distance precedes recorded goal marker onset"
            if reason not in log["completionReasons"]:
                log["completionReasons"].append(reason)
        log.update({"relativePath": relative, "sourceFolder": path.parent.relative_to(root).as_posix(),
                    "logNumber": int(path.stem) if path.stem.isdigit() else path.stem,
                    "sizeBytes": stat.st_size})
        entries[relative] = {"stamp": stamp, "log": log}
        inventory.append({"logNumber": log["logNumber"], "sourceFolder": log["sourceFolder"],
                          "relativePath": relative, "sizeBytes": stat.st_size, "rowCount": log["rowCount"],
                          "columnCount": len(log["columns"]), "columns": json.dumps(log["columns"]),
                          "metadata": json.dumps(log["metadata"], ensure_ascii=False),
                          "gitCommit": log["metadata"].get("gitCommit"),
                          "buildDate": log["metadata"].get("buildDate"),
                          "buildTime": log["metadata"].get("buildTime"),
                          "sha256": log["sha256"], "badRows": log["badRows"],
                          "distanceBacksteps": log.get("distanceBacksteps", 0),
                          "distanceMaxCorrectionPulse": log.get("distanceMaxCorrectionPulse"),
                          "finalCntlog": log.get("finalCntlog"), "nulBytes": log.get("nulBytes", 0),
                          "encoding": log["encoding"], "timeWraps": log.get("timeWraps", 0)})
        logs.append(log)
        if (index + 1) % 500 == 0:
            print(f"inventory {index + 1}/{len(paths)}", flush=True)
            if dirty_cache:
                write_cache(cache_path, root, entries)
    if dirty_cache or len(entries) != len(cached.get("entries", {})):
        write_cache(cache_path, root, entries)
    write_csv(out / "log_inventory.csv", inventory)
    return logs


def representatives(logs, count=16):
    result = {}
    for course in sorted({v["sourceFolder"] for v in logs if v["sourceFolder"] != "."}):
        candidates = [v for v in logs if v["sourceFolder"] == course and v["feature"]
                      and v["completionStatus"] == "NORMAL_CANDIDATE"
                      and meta(v, "optimalTrace") not in (3, 4)]
        selected = [v for n in SEEDS.get(course, []) for v in candidates if v["logNumber"] == n]
        # モード・年代を横断する複数代表。異常ログを代表に使わない。
        groups = collections.defaultdict(list)
        for v in candidates:
            groups[str(meta(v, "optimalTrace"))].append(v)
        for group in groups.values():
            for index in np.linspace(0, len(group)-1, min(count // max(len(groups), 1), len(group)), dtype=int):
                if group[index] not in selected:
                    selected.append(group[index])
        result[course] = selected
    return result


def match(log, refs, exclude_self=False):
    matches = []
    if not log["feature"]:
        return matches
    variants = [("same", log["feature"]), ("reverse", reverse(log["feature"])),
                ("reverse_swap", reverse(log["feature"], True))]
    for course, examples in refs.items():
        options = [(compare_variants(variants, ref["feature"]), ref) for ref in examples
                   if not exclude_self or ref["relativePath"] != log["relativePath"]]
        if options:
            (score, direction, parts), ref = max(options, key=lambda v: v[0][0])
            matches.append((score, course, direction, parts, ref))
    return sorted(matches, key=lambda v: v[0], reverse=True)


def auto_eligible(best, margin):
    score, _, _, parts, _ = best
    return (score >= .86 and margin >= .06 and parts["markerSequence"] == 1
            and (parts["markerPosition"] or 0) >= .80 and parts["distance"] >= .94
            and (parts["turn"] or 0) >= .65 and (parts["gyro"] or 0) >= .75)


FIELDS = ["logNumber", "sourceFolder", "status", "classifiedCourse", "direction", "confidence",
          "completionStatus", "finalDistance", "rowCount", "emcStop", "optimalTrace",
          "routeSourceLog", "analysisSourceLog", "gyroScore", "rocScore", "markerScore", "turnScore",
          "reason", "relativePath", "referenceLog", "scoreMargin", "markerSequenceScore",
          "distanceScore", "finalDistancePulse", "distanceColumn", "maxAngularVelocity", "unknownCluster",
          "relativeDirection", "directionAnchor", "finalCntlog", "distanceBacksteps", "distanceMaxCorrectionPulse"]


def classify(logs, refs):
    results = []
    for index, log in enumerate(logs):
        existing = log["sourceFolder"] != "."
        row = dict.fromkeys(FIELDS, "")
        row.update({k: log.get(k, "") for k in ("logNumber", "sourceFolder", "completionStatus",
                    "finalDistance", "rowCount", "relativePath", "finalDistancePulse", "distanceColumn",
                    "finalCntlog", "distanceBacksteps", "distanceMaxCorrectionPulse")})
        for key in ("emcStop", "optimalTrace", "routeSourceLog", "analysisSourceLog"):
            row[key] = log["metadata"].get(key, "")
        row["status"] = "EXISTING" if existing else "UNCLASSIFIED"
        row["classifiedCourse"] = log["sourceFolder"] if existing else ""
        reasons = list(log["completionReasons"])
        if log["feature"]:
            row["maxAngularVelocity"] = log["feature"]["maxAngularVelocity"]
        matches = match(log, refs, exclude_self=existing) if meta(log, "optimalTrace") not in (3, 4) else []
        if matches:
            best = matches[0]
            score, course, direction, parts, ref = best
            margin = score - matches[1][0] if len(matches) > 1 else score
            row.update(confidence=round(score, 5), direction=direction, referenceLog=ref["logNumber"],
                       scoreMargin=round(margin, 5), gyroScore=parts["gyro"], rocScore=parts["roc"],
                       markerScore=parts["markerPosition"], turnScore=parts["turn"],
                       markerSequenceScore=parts["markerSequence"], distanceScore=parts["distance"])
            row["predictedCourse"] = course
            if not existing and log["completionStatus"] != "ABNORMAL":
                if score >= .70:
                    row["classifiedCourse"] = course
                    row["status"] = "REVIEW"
                    if log["completionStatus"] == "NORMAL_CANDIDATE" and auto_eligible(best, margin):
                        row["status"] = "AUTO"
                    else:
                        reasons.append("insufficient independent agreement or ambiguous course")
                else:
                    reasons.append("no known course match")
            if existing and course != log["sourceFolder"]:
                reasons.append(f"existing label disagrees with prediction={course}")
        else:
            reasons.append("no usable features/reference")
        if meta(log, "optimalTrace") in (3, 4):
            if not existing and log["completionStatus"] != "ABNORMAL":
                row["status"] = "REVIEW"
                row["classifiedCourse"] = ""
            reasons.append("PATH/SHORTCUT requires parent validation")
        row["reason"] = "; ".join(reasons)
        results.append(row)
        if (index + 1) % 1000 == 0:
            print(f"classify {index + 1}/{len(logs)}", flush=True)
    # 親番号重複は継承不可。複数段の親も固定点まで伝播。
    by_number = collections.defaultdict(list)
    for log, row in zip(logs, results):
        by_number[log["logNumber"]].append((log, row))
    for _ in range(len(logs)):
        changed = False
        for log, row in zip(logs, results):
            if meta(log, "optimalTrace") not in (3, 4) or row["status"] == "AUTO" or row.get("_parentAuditDone"):
                continue
            if log["completionStatus"] == "ABNORMAL":
                continue
            ids = [int(v) for k in ("routeSourceLog", "analysisSourceLog")
                   for v in [meta(log, k)] if v is not None and v > 0 and v.is_integer()]
            parents = [by_number[n][0] for n in set(ids) if len(by_number[n]) == 1]
            proven = [(p, r) for p, r in parents if r["status"] in ("AUTO", "EXISTING")
                      and p["completionStatus"] == "NORMAL_CANDIDATE"]
            if not proven:
                continue
            courses = {r["classifiedCourse"] for _, r in proven}
            if len(courses) != 1 or len(proven) != len(set(ids)):
                continue
            ratios = [min(log["finalDistancePulse"] or 0, p["finalDistancePulse"] or 0) /
                      max(log["finalDistancePulse"] or 1, p["finalDistancePulse"] or 1) for p, _ in proven]
            course = next(iter(courses))
            row["predictedCourse"] = course
            if row["sourceFolder"] != ".":
                row["_parentAuditDone"] = True
                row["confidence"] = round(min(ratios), 5)
                row["reason"] = "existing PATH/SHORTCUT audited against validated parent"
                if course != row["classifiedCourse"]:
                    row["reason"] += f"; existing label disagrees with parent={course}"
                if min(ratios) < .92:
                    row["reason"] += "; parent distance mismatch"
                continue
            row["classifiedCourse"] = course
            if min(ratios) >= .92 and log["completionStatus"] == "NORMAL_CANDIDATE":
                row["status"] = "AUTO"
                row["confidence"] = round(min(ratios), 5)
                row["reason"] = "validated parent course inheritance; no recorded stop/fault; missing diagnostics are unverified"
                changed = True
            else:
                row["status"] = "REVIEW"
                row["reason"] += "; parent distance mismatch or completion pending"
        if not changed:
            break
    return results


def unknown_clusters(logs, results):
    clusters = []
    for log, row in zip(logs, results):
        if (row["status"] != "UNCLASSIFIED" or row["completionStatus"] != "NORMAL_CANDIDATE"
                or not log["feature"] or meta(log, "optimalTrace") in (3, 4)):
            continue
        found = None
        for cluster in clusters:
            best = compare(log["feature"], cluster[0][0]["feature"])
            if best[0] >= .90 and best[2]["markerSequence"] == 1 and (best[2]["markerPosition"] or 0) >= .85 and best[2]["distance"] >= .96:
                found = cluster
                break
        if found is None:
            found = []
            clusters.append(found)
        found.append((log, row))
    for index, cluster in enumerate(clusters, 1):
        name = f"unknown_cluster_{index:03d}"
        for _, row in cluster:
            row["unknownCluster"] = name
    return clusters


def cluster_preview(clusters, out):
    # スタンドアロンHTML/SVGで代表形状とマーカーを確認できる。
    panels = ["<!doctype html><meta charset='utf-8'><title>未知コース候補</title>",
              "<h1>未知コース候補</h1><p>クラスタは仮説です。確認後にコース名を登録してください。</p>"]
    manifest = []
    for index, cluster in enumerate(clusters, 1):
        log = cluster[0][0]
        f = log["feature"]
        name = f"unknown_cluster_{index:03d}"
        manifest.append({"cluster": name, "count": len(cluster), "representativeLog": log["logNumber"],
                         "members": json.dumps([v[0]["relativePath"] for v in cluster]), "course": ""})
        values = f["gyro"] or [0] * N
        limit = max(max(abs(v) for v in values), .001)
        points = " ".join(f"{i*700/(N-1):.1f},{100-v/limit*80:.1f}" for i, v in enumerate(values))
        markers = "".join(f"<line x1='{p*700:.1f}' x2='{p*700:.1f}' y1='0' y2='200' stroke='#c53'/><text x='{p*700:.1f}' y='15'>{v}</text>" for p, v in (f["events"] or []))
        panels.append(f"<h2>{name}: {len(cluster)}本 / 代表{log['logNumber']}</h2><p>距離 {f['distance']:.0f} pulse</p><svg viewBox='0 0 720 210' width='720' role='img' aria-label='距離基準回転とマーカー'><polyline points='{points}' fill='none' stroke='#246'/>{markers}</svg>")
    (out / "unknown_clusters.html").write_text("\n".join(panels), encoding="utf-8")
    write_csv(out / "unknown_clusters.csv", manifest, ["cluster", "count", "representativeLog", "members", "course"])


def assign_directions(root, refs, results, out):
    """既知CW/CCWログと独立照合し、裏付けのない絶対方向は空欄にする。"""
    anchors = []
    for n in list(range(12838, 12843)) + list(range(12844, 12849)):
        paths = [root / f"{n}.csv", root.parent / f"{n}.csv"]
        paths.extend(root.glob(f"course_*/{n}.csv"))
        path = next((p for p in paths if p.is_file()), None)
        if path is None:
            continue
        log = parse_csv(path)
        state, _ = completion(log)
        f = features(log)
        if state == "NORMAL_CANDIDATE" and f:
            anchors.append((n, "CW" if n <= 12842 else "CCW", f, log["sha256"], str(path)))
    known = {n: direction for n, direction, _, _, _ in anchors}
    ref_directions = {}
    def stable_markers(f):
        result = {k: v for k, v in f.items() if not k.startswith("_")}
        if f["markerColumn"] == "courseMarker" and f["events"] is not None:
            # 旧/新のLEFT記録差を方向推定へ持ち込まない。SG/CROSSで確認。
            result["events"] = [event for event in f["events"] if event[1] in (1, 3)]
        return result
    for examples in refs.values():
        for ref in examples:
            options = [(compare(stable_markers(ref["feature"]), stable_markers(f)), n, direction)
                       for n, direction, f, _, _ in anchors]
            if not options:
                continue
            (score, relative, parts), n, direction = max(options, key=lambda v: v[0][0])
            if score >= .90 and parts["markerSequence"] == 1 and (parts["markerPosition"] or 0) >= .85:
                if relative != "same":
                    direction = "CCW" if direction == "CW" else "CW"
                ref_directions[ref["logNumber"]] = (direction, n)
    for row in results:
        relative = row["direction"]
        row["relativeDirection"], row["direction"] = relative, ""
        if row["logNumber"] in known:
            row["direction"] = known[row["logNumber"]]
            row["directionAnchor"] = row["logNumber"]
        elif row["referenceLog"] in ref_directions and float(row["confidence"] or 0) >= .86:
            direction, anchor = ref_directions[row["referenceLog"]]
            row["direction"] = direction if relative == "same" else ("CCW" if direction == "CW" else "CW")
            row["directionAnchor"] = anchor
    write_csv(out / "direction_anchors.csv", [{"logNumber": n, "direction": d, "sha256": h, "path": p}
                                              for n, d, _, h, p in anchors], ["logNumber", "direction", "sha256", "path"])
    checks = []
    for n, direction, f, _, _ in anchors:
        for m, other_direction, other, _, _ in anchors:
            if direction != "CW" or other_direction != "CCW":
                continue
            score, relative, parts = compare(f, other)
            checks.append({"cwLog": n, "ccwLog": m, "score": score, "relativeDirection": relative,
                           "gyroScore": parts["gyro"], "markerSequenceScore": parts["markerSequence"],
                           "markerPositionScore": parts["markerPosition"],
                           "pass": relative != "same" and score >= .85})
    write_csv(out / "direction_validation.csv", checks,
              ["cwLog", "ccwLog", "score", "relativeDirection", "gyroScore", "markerSequenceScore", "markerPositionScore", "pass"])


def register_clusters(path, clusters):
    """人がcourse列へ記入したクラスタ表を、代表・全メンバー一致確認後に登録する。"""
    current = {f"unknown_cluster_{i:03d}": c for i, c in enumerate(clusters, 1)}
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assigned = set()
    for row in rows:
        course = row.get("course", "").strip()
        if not course:
            continue
        if not re.fullmatch(r"course_\d{3}", course) or int(course[-3:]) < 6:
            raise ValueError("registered unknown course must be course_006 or later")
        cluster = current.get(row["cluster"])
        if not cluster or str(cluster[0][0]["logNumber"]) != row["representativeLog"]:
            raise ValueError("cluster representative changed; review new cluster table")
        if json.loads(row["members"]) != [log["relativePath"] for log, _ in cluster]:
            raise ValueError("cluster members changed; review new cluster table")
        for _, result in cluster:
            result.update(status="AUTO", classifiedCourse=course,
                          reason=f"explicit user cluster registration: {row['cluster']}")
        assigned.add(course)
    return assigned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--refresh", action="store_true", help="全ファイルを再読込。キャッシュはサイズ/mtimeで再利用")
    parser.add_argument("--cluster-map", type=Path, help="確認済み未知クラスタ表（course列へcourse_006以降を記入）")
    args = parser.parse_args()
    root, out = args.root.resolve(), args.output.resolve()
    if not root.is_dir():
        parser.error(f"input directory does not exist: {root}")
    if out == root or root in out.parents:
        parser.error("output must be outside input tree")
    out.mkdir(parents=True, exist_ok=True)
    logs = load_logs(root, out, args.refresh)
    refs = representatives(logs)
    write_csv(out / "representatives.csv", [{"course": c, "logNumber": v["logNumber"], "relativePath": v["relativePath"],
                "optimalTrace": meta(v, "optimalTrace")} for c, group in refs.items() for v in group])
    results = classify(logs, refs)
    clusters = unknown_clusters(logs, results)
    assigned = register_clusters(args.cluster_map.resolve(), clusters) if args.cluster_map else set()
    cluster_preview(clusters, out)
    assign_directions(root, refs, results, out)
    validation = []
    for row in results:
        if row["status"] == "EXISTING":
            validation.append({"logNumber": row["logNumber"], "expectedCourse": row["sourceFolder"],
                               "predictedCourse": row.get("predictedCourse", ""),
                               "agreement": row.get("predictedCourse") == row["sourceFolder"],
                               "confidence": row["confidence"], "completionStatus": row["completionStatus"],
                               "autoEligible": number(row["optimalTrace"]) not in (3, 4) and row["completionStatus"] == "NORMAL_CANDIDATE" and
                               float(row["confidence"] or 0) >= .86 and float(row["scoreMargin"] or 0) >= .06 and
                               row["markerSequenceScore"] == 1 and float(row["markerScore"] or 0) >= .80 and
                               float(row["distanceScore"] or 0) >= .94 and float(row["turnScore"] or 0) >= .65 and
                               float(row["gyroScore"] or 0) >= .75,
                               "reason": row["reason"]})
    write_csv(out / "validation.csv", validation)
    # コース別に独立した既存ラベル照合で自動判定精度を検査。
    gates = {}
    for course in refs:
        eligible = [r for r in validation if r["predictedCourse"] == course and r["autoEligible"]]
        gates[course] = len(eligible) >= 5 and all(r["agreement"] for r in eligible)
    gates.update({course: True for course in assigned})
    for row in results:
        if row["status"] == "AUTO" and not gates.get(row["classifiedCourse"], False):
            row["status"] = "REVIEW"
            row["reason"] += "; course validation gate insufficient (<5) or false positive"
    write_csv(out / "classification.csv", results, FIELDS)
    review = [r for r in results if r["status"] in ("REVIEW", "UNCLASSIFIED") or r["completionStatus"] == "ABNORMAL"
              or "disagrees" in r["reason"] or "parent distance mismatch" in r["reason"]]
    write_csv(out / "review.csv", review, FIELDS)
    move_fields = ["source", "destination", "status", "logNumber", "sha256", "sizeBytes", "confidence"]
    plan = [{"source": log["relativePath"], "destination": f"{row['classifiedCourse']}/{Path(log['relativePath']).name}",
             "status": "AUTO", "logNumber": log["logNumber"], "sha256": log["sha256"],
             "sizeBytes": log["sizeBytes"], "confidence": row["confidence"]}
            for log, row in zip(logs, results) if row["status"] == "AUTO" and row["sourceFolder"] == "."]
    write_csv(out / "move_plan.csv", plan, move_fields)
    (out / "move_plan.json").write_text(json.dumps({"root": str(root), "planSha256": hashlib.sha256((out / "move_plan.csv").read_bytes()).hexdigest()}, indent=2), encoding="utf-8")
    status = collections.Counter(r["status"] for r in results)
    completion_counts = collections.Counter(r["completionStatus"] for r in results)
    schema = collections.Counter(tuple(log["columns"]) for log in logs)
    write_csv(out / "schema_summary.csv", [{"count": count, "columnCount": len(cols), "columns": json.dumps(cols)}
                                           for cols, count in schema.most_common()])
    summary = ["# 全ログ分類結果", "", f"総ログ数: {len(logs)}", f"既存分類済み: {status['EXISTING']}",
               f"新規自動分類: {status['AUTO']}", f"要確認: {status['REVIEW']}",
               f"未分類: {status['UNCLASSIFIED']}", f"異常・分類無効候補（既存を含む）: {completion_counts['ABNORMAL']}",
               f"完走判定保留: {completion_counts['PENDING']}", f"未知コース候補: {len(clusters)}クラスタ / {sum(map(len, clusters))}本",
               f"スキーマ種類: {len(schema)}", "", "## 既存ラベル照合（自身との比較を除外）", ""]
    for course in refs:
        tested = [r for r in validation if r["expectedCourse"] == course]
        correct = sum(r["agreement"] for r in tested)
        eligible = [r for r in validation if r["predictedCourse"] == course and r["autoEligible"]]
        count = sum(r["classifiedCourse"] == course and r["status"] in ("AUTO", "EXISTING") for r in results)
        candidates = sum(r["classifiedCourse"] == course and r["status"] == "REVIEW" for r in results)
        summary.append(f"- {course}: 既存+AUTO {count}本、REVIEW候補 {candidates}本、照合一致 {correct}/{len(tested)}、AUTO基準適合 {len(eligible)}本、移動候補出力許可 {gates[course]}")
    with (out / "direction_validation.csv").open(encoding="utf-8-sig", newline="") as stream:
        direction_checks = list(csv.DictReader(stream))
    if direction_checks:
        passed = sum(v["pass"] == "True" for v in direction_checks)
        minimum = min(float(v["score"]) for v in direction_checks)
        summary += ["", f"既知CW/CCW反転比較: {passed}/{len(direction_checks)}組通過、最小一致スコア {minimum:.3f}。"]
    summary += ["", "## 判定の限界と実行", "", "実ファイルは移動していない。AUTOのみ移動候補に含める。",
                "NORMAL_CANDIDATEは記録された情報に矛盾がない候補であり、実機完走の独立証明ではない。",
                "ABNORMALは停止・破損・解析用の距離条件違反などを含む分類除外候補。距離条件違反だけで実機異常終了と断定しない。",
                "古いログの欠落メタデータは推測しない。emcStop欠落は判定保留。closureReason=8、およびPATH系reason=1単独は異常扱いしない。",
                "分類の距離比較はpulse。finalDistanceは現行58019 pulse/mによる参考mmで、過去機体の実寸法を保証しない。",
                "補正距離の一時的な戻りは、一次走行以外で総距離の10%以内に限り単調包絡で特徴量化する。戻り回数と最大補正量を出力し、モード欠落の完走判定は保留のまま。",
                "directionは既知CW/CCWアンカーと十分一致した代表からの推定（未確認は空欄）。relativeDirectionは代表に対するsame/reverse/reverse_swap。directionAnchorが根拠番号。アンカーはoldと親v2から読み取り、分類対象数には含めない。",
                "gyroScoreは速度で割った距離当たり回転の相関。ROCは逆数とgyro符号による曲率形状。欠落項目は空欄。",
                "PATH系は正常な確定親・距離比92%以上・記録に異常がない場合のみ継承。pathStateが存在しなければ状態の健全性は検証できない。",
                "未知クラスタは代表との一致に基づく仮説（単独も含む）。unknown_clusters.htmlで代表を確認し、手動でコース名を決定する。",
                "未知クラスタ登録: unknown_clusters.csvを別名へコピーしcourse列にcourse_006以降を記入、--cluster-mapで指定する。代表と全メンバーが一致したクラスタを明示登録としてAUTOへ昇格する（移動は別実行）。",
                "既存分類内のAUTO基準適合は自身を除く照合。ただしラベル付き代表を用いるため外部検証セットの精度ではない。",
                "キャッシュはsize/mtimeで再利用。入力変更に疑義がある場合は--refresh。移動時はSHA256を再検証する。",
                "", "```powershell", "python analysis/script/classify_logs.py", "python analysis/script/apply_classification.py --dry-run", "```"]
    (out / "classification_summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(json.dumps({"status": status, "completion": completion_counts, "gates": gates}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
