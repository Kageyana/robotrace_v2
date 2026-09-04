"""Summarize the latest gain-isolation run (logs 12225-12229)."""

from __future__ import annotations

import csv
from pathlib import Path

from analyze_path_gains_12205_12224 import LOG_DIR, summarize


OUT_DIR = Path(__file__).resolve().parents[1] / "path_gains_12225_12229"


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summaries = [summarize(number) for number in range(12225, 12230)
                 if (LOG_DIR / f"{number}.csv").exists()]
    output = OUT_DIR / "path_gains_12225_12229_summary.csv"
    fields = list(summaries[0]) if summaries else []
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
    print(output)
    for row in summaries:
        print(row)


if __name__ == "__main__":
    main()
