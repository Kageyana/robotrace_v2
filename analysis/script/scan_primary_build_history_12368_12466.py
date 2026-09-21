#!/usr/bin/env python3
"""Index primary-run builds and marker positions for a numeric log range."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


PULSES_PER_MM = 53.424


def scan(path: Path) -> dict[str, str] | None:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        first = stream.readline().strip()
        if "=" not in first:
            return None
        metadata = dict(field.split("=", 1) for field in first.split(",") if "=" in field)
        if float(metadata.get("optimalTrace", "-1")) != 0:
            return None
        columns = stream.readline().strip().split(",")
        if not {"cntlog", "encTotalOptimal", "courseMarker"}.issubset(columns):
            return None
        first_left = None
        last = None
        for row in csv.DictReader(stream, fieldnames=columns):
            if not row.get("cntlog"):
                continue
            last = row
            if first_left is None and row.get("courseMarker") == "2":
                distance = float(row["encTotalOptimal"]) / PULSES_PER_MM
                if distance > 100:
                    first_left = distance
        if last is None:
            return None
        return {
            "log": path.stem,
            "commit": metadata.get("gitCommit", ""),
            "buildDate": metadata.get("buildDate", ""),
            "buildTime": metadata.get("buildTime", ""),
            "records": metadata.get("logExpectedRows", ""),
            "distanceEnd_mm": f'{float(last["encTotalOptimal"]) / PULSES_PER_MM:.0f}',
            "firstLeftLog_mm": "" if first_left is None else f"{first_left:.0f}",
            "emcStop": metadata.get("emcStop", ""),
            "closureValid": metadata.get("closureValid", ""),
            "omegaKpEnd": metadata.get("lineTraceOmegaFBCtrl.kp", ""),
            "omegaKpStart": metadata.get("runStartOmega.kp", ""),
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--start", type=int, default=12368)
    parser.add_argument("--end", type=int, default=12466)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.start > args.end:
        parser.error("--start must not exceed --end")
    output = args.output or (Path(__file__).resolve().parents[1] /
                             f"log_{args.start}_{args.end}_build_history.csv")
    results = []
    for run_id in range(args.start, args.end + 1):
        path = args.log_dir / f"{run_id}.csv"
        if path.is_file():
            result = scan(path)
            if result is not None:
                results.append(result)
    if not results:
        parser.error("No primary-run CSV logs found")
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print(output)
    print(f"Indexed {len(results)} primary runs")


if __name__ == "__main__":
    main()
