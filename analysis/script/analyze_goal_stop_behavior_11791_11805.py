"""Summarize the last logged control state before the goal-stop phase."""

import csv
from pathlib import Path


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT = Path(r"D:\robotrace\robotrace_v2\analysis\log_11791_11805_goal_stop_last_logged.csv")


def parse_header(header):
    columns = []
    params = {}
    for index, field in enumerate(header):
        if "=" in field:
            key, value = field.split("=", 1)
            params[key] = value
        else:
            columns.append(field)
    return columns, params


def load(number):
    with (LOG_DIR / f"{number}.csv").open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        columns, params = parse_header(next(reader))
        rows = [dict(zip(columns, row[:len(columns)])) for row in reader if len(row) >= len(columns)]
    return params, rows


def main():
    output = []
    for number in range(11791, 11806):
        params, rows = load(number)
        if params.get("optimalTrace") != "2.00" or not rows:
            continue
        last = rows[-1]
        output.append(
            {
                "log": number,
                "autoStart": params.get("autoStart", ""),
                "emcStop": params.get("emcStop", ""),
                "startVoltage_V": params.get("batteryVoltage_V", ""),
                "last_cntlog_ms": last.get("cntlog", ""),
                "last_courseMarker": last.get("courseMarker", ""),
                "last_optimalIndex": last.get("optimalIndex", ""),
                "last_targetSpeed": last.get("targetSpeed", ""),
                "last_lineTraceCtrl": last.get("lineTraceCtrl", ""),
                "last_targetAngularvelo": last.get("targetAngularvelo", ""),
                "last_motorpwmL": last.get("motorpwmL", ""),
                "last_motorpwmR": last.get("motorpwmR", ""),
            }
        )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    print(f"output={OUT}")
    for row in output:
        print(
            f"log={row['log']} auto={row['autoStart']} cnt={row['last_cntlog_ms']} "
            f"marker={row['last_courseMarker']} target={row['last_targetSpeed']} "
            f"line={row['last_lineTraceCtrl']} yawTarget={row['last_targetAngularvelo']} "
            f"motor={row['last_motorpwmL']}/{row['last_motorpwmR']}"
        )


if __name__ == "__main__":
    main()
