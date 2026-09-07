#!/usr/bin/env python3
"""12345から生成した3経路をMatplotlibの同一座標軸へ重ねて描画する。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt

from compare_shortcut_routes_12345 import (
    build_racing_line,
    curve_mask,
    expand_anchors,
    read_raw,
    resample,
    robust_radius,
    route_length,
    smooth_route,
)


def xy(route: list[list[int]]) -> tuple[list[int], list[int]]:
    return [point[0] for point in route], [point[1] for point in route]


def draw_routes(axis, source, current, proposed, *, labels: bool) -> None:
    source_x, source_y = xy(source)
    current_x, current_y = xy(current)
    proposed_x, proposed_y = xy(proposed)
    axis.plot(source_x, source_y, color="#4b5563", linewidth=3.2, linestyle=(0, (5, 3)),
              label="一次走行経路（Level 0）" if labels else None, zorder=1)
    axis.plot(current_x, current_y, color="#dc2626", linewidth=1.8,
              label="現行Elastic Band候補（棄却）" if labels else None, zorder=2)
    axis.plot(proposed_x, proposed_y, color="#2563eb", linewidth=1.6,
              label="新方式案（直線固定・カーブ変形）" if labels else None, zorder=3)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-show", action="store_true",
                        help="PNGだけを保存し、Matplotlibの操作ウィンドウを開かない")
    args = parser.parse_args()

    source, marker_anchors = resample(read_raw(args.log))
    anchors = expand_anchors(marker_anchors)
    current = smooth_route(source, [0.0 if anchor else 1.0 for anchor in anchors])
    curves = curve_mask(source)
    proposed = build_racing_line(source, curves)

    source_length = route_length(source)
    current_reduction = (1.0 - route_length(current) / source_length) * 100.0
    proposed_reduction = (1.0 - route_length(proposed) / source_length) * 100.0
    radius_before = robust_radius(source, curves)
    radius_after = robust_radius(proposed, curves)

    plt.rcParams.update({
        "font.family": "Yu Gothic",
        "axes.unicode_minus": False,
        "font.size": 11,
    })
    figure, axis = plt.subplots(figsize=(10.5, 9.0), constrained_layout=True)
    if figure.canvas.manager is not None:
        figure.canvas.manager.set_window_title("一次走行12345 Level 1経路比較")
    draw_routes(axis, source, current, proposed, labels=True)
    axis.set_title("一次走行12345：Level 1経路生成方法の重ね合わせ", fontsize=16)
    axis.set_xlabel("X [mm]")
    axis.set_ylabel("Y [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linewidth=0.5, alpha=0.35)
    axis.legend(loc="upper left", framealpha=0.92)

    metrics = (
        f"現行候補: 短縮率 {current_reduction:.3f}% → 0.5%条件未達で棄却\n"
        f"新方式案: 短縮率 {proposed_reduction:.3f}% / 直線変位 0 mm\n"
        f"新方式案: 曲率半径指標 R95 {radius_before:.1f} → {radius_after:.1f} mm"
    )
    axis.text(0.02, 0.02, metrics, transform=axis.transAxes, va="bottom", ha="left",
              bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "#9ca3af",
                    "alpha": 0.92})

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    print(args.output)
    if not args.no_show:
        plt.show()
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
