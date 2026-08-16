"""Compute voltage-command and control metrics for logs 11402-11421."""

import csv
import math
from pathlib import Path
from statistics import mean


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_PATH = Path(r"D:\robotrace\robotrace_v2\analysis\log_11402_11421_voltage_command_metrics.csv")
PULSE_METER = 53424.0


def parse_header(row):
    columns = []
    params = {}
    for cell in row:
        if not cell:
            continue
        if "=" in cell:
            key, value = cell.split("=", 1)
            params[key] = value
        else:
            columns.append(cell)
    return columns, params


def parse_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rms(values):
    values = [value for value in values if not math.isnan(value)]
    return math.sqrt(mean(value * value for value in values)) if values else math.nan


def p95_abs(values):
    values = sorted(abs(value) for value in values if not math.isnan(value))
    return values[int(0.95 * (len(values) - 1))] if values else math.nan


def metric_row(number):
    with (LOG_DIR / f"{number}.csv").open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        columns, params = parse_header(next(reader))
        rows = [dict(zip(columns, row[: len(columns)])) for row in reader if len(row) >= len(columns)]

    line_values = [parse_float(row.get("lineTraceCtrl")) for row in rows]
    gyro_values = [parse_float(row.get("gyroVal_Z")) for row in rows]
    speed_errors = []
    for row in rows:
        target = parse_float(row.get("targetSpeed"))
        current = parse_float(row.get("encCurrentN"))
        if not math.isnan(target) and not math.isnan(current):
            speed_errors.append((target - current) / PULSE_METER * 1000.0)
    pwm_sat = [
        max(abs(parse_float(row.get("motorpwmL"))), abs(parse_float(row.get("motorpwmR")))) >= 1000.0
        for row in rows
    ]
    command_values = [
        value
        for row in rows
        for value in (
            parse_float(row.get("motorVoltageCmdL_mV")),
            parse_float(row.get("motorVoltageCmdR_mV")),
        )
        if not math.isnan(value)
    ]
    battery_values = [
        parse_float(row.get("batteryVoltage_mV")) / 1000.0
        for row in rows
        if not math.isnan(parse_float(row.get("batteryVoltage_mV")))
    ]
    return {
        "log": number,
        "autoStart": params.get("autoStart", ""),
        "optimalTrace": params.get("optimalTrace", ""),
        "emcStop": params.get("emcStop", ""),
        "batteryHeader_V": params.get("batteryVoltage_V", ""),
        "batteryMean_V": mean(battery_values) if battery_values else math.nan,
        "lineTraceCtrl_rms": rms(line_values),
        "lineTraceCtrl_p95_abs": p95_abs(line_values),
        "gyroVal_Z_rms": rms(gyro_values),
        "gyroVal_Z_p95_abs": p95_abs(gyro_values),
        "speedErr_mean_mps": mean(speed_errors) if speed_errors else math.nan,
        "speedErr_abs_p95_mps": p95_abs(speed_errors),
        "motorDutySat_pct": 100.0 * sum(pwm_sat) / len(pwm_sat) if pwm_sat else math.nan,
        "motorVoltageCmd_abs_max_mV": max(abs(value) for value in command_values) if command_values else math.nan,
    }


def main():
    rows = [metric_row(number) for number in range(11402, 11422)]
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUT_PATH.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(OUT_PATH)


if __name__ == "__main__":
    main()
