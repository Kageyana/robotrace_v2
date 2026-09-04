"""Create comparison plots for the gain change around logs 12215 and 12220."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt

from analyze_path_gains_12205_12224 import LOG_DIR, load_log


OUT_DIR = Path(__file__).resolve().parents[1] / "path_gains_12205_12224"
BEFORE = [12216, 12217, 12218, 12219]
AFTER = [12221, 12222, 12223, 12224]


def rows(number: int):
    _, _, data = load_log(number)
    return data


def num(data: dict[str, str], key: str) -> float:
    return float(data[key])


def plot_xy() -> None:
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    for number in BEFORE:
        data = rows(number)
        ax.plot([num(r, "x") for r in data], [num(r, "y") for r in data],
                color="tab:blue", alpha=0.45, linewidth=1.0)
    for number in AFTER:
        data = rows(number)
        ax.plot([num(r, "x") for r in data], [num(r, "y") for r in data],
                color="tab:red", alpha=0.45, linewidth=1.0)
    ax.set_title("PATH REPLAY XY: before (15/6 assumed) vs after (10/16)")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.savefig(OUT_DIR / "path_gains_12215_vs_12220_xy.png", dpi=160)
    plt.close(fig)


def plot_errors() -> None:
    fig, (ax_y, ax_h) = plt.subplots(2, 1, figsize=(9, 7), sharex=True,
                                     constrained_layout=True)
    for number, color, label in [(12218, "tab:blue", "before"), (12223, "tab:red", "after")]:
        data = rows(number)
        distance = [num(r, "encTotalOptimal") / 54.324 for r in data]
        ax_y.plot(distance, [abs(num(r, "pathErrorY_mm")) for r in data],
                  color=color, label=f"{number} ({label})")
        ax_h.plot(distance, [abs(num(r, "pathErrorHeading_cdeg")) / 100.0 for r in data],
                  color=color, label=f"{number} ({label})")
    ax_y.set_ylabel("|pathErrorY| [mm]")
    ax_h.set_ylabel("|heading error| [deg]")
    ax_h.set_xlabel("encTotalOptimal distance [mm]")
    ax_y.grid(True, alpha=0.3)
    ax_h.grid(True, alpha=0.3)
    ax_y.legend()
    ax_h.legend()
    fig.savefig(OUT_DIR / "path_gains_12215_vs_12220_errors.png", dpi=160)
    plt.close(fig)


def plot_yaw_speed() -> None:
    fig, (ax_yaw, ax_speed) = plt.subplots(2, 1, figsize=(9, 7), sharex=True,
                                            constrained_layout=True)
    for number, color, label in [(12218, "tab:blue", "before"), (12223, "tab:red", "after")]:
        data = rows(number)
        time = [num(r, "cntlog") / 1000.0 for r in data]
        ax_yaw.plot(time, [num(r, "targetAngularvelo") for r in data],
                    color=color, alpha=0.8, label=f"target {number} ({label})")
        ax_yaw.plot(time, [num(r, "gyroVal_Z") for r in data],
                    color=color, linestyle="--", alpha=0.7, label=f"gyro {number} ({label})")
        ax_speed.plot(time, [num(r, "targetSpeed") for r in data],
                      color=color, label=f"target {number} ({label})")
        ax_speed.plot(time, [num(r, "encCurrentN") for r in data],
                      color=color, linestyle="--", alpha=0.7, label=f"encoder {number} ({label})")
    ax_yaw.set_ylabel("yaw rate [deg/s]")
    ax_speed.set_ylabel("speed [pulse/ms]")
    ax_speed.set_xlabel("cntlog time [s]")
    ax_yaw.grid(True, alpha=0.3)
    ax_speed.grid(True, alpha=0.3)
    ax_yaw.legend(ncol=2, fontsize=8)
    ax_speed.legend(ncol=2, fontsize=8)
    fig.savefig(OUT_DIR / "path_gains_12215_vs_12220_yaw_speed.png", dpi=160)
    plt.close(fig)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plot_xy()
    plot_errors()
    plot_yaw_speed()
    print(OUT_DIR)


if __name__ == "__main__":
    main()
