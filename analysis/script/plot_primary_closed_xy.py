"""一次走行のジャイロXYと経路用XYをスキーマ別に描く。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def read_log(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    with path.open(encoding="utf-8", newline="") as stream:
        metadata_line = stream.readline().strip()
        metadata = dict(item.split("=", 1) for item in metadata_line.split(",") if "=" in item)
        rows = list(csv.DictReader(stream))
    version = metadata.get("logSchemaVersion")
    route_fields = {"x_fused_mm", "y_fused_mm"} if version in ("9", "10") else {"x_closed_mm", "y_closed_mm"}
    required = {"encTotalOptimal", "x", "y"} | route_fields
    if version not in ("7", "8", "9", "10") or not required.issubset(rows[0] if rows else {}):
        raise ValueError("スキーマ7/8/9/10の一次走行CSVと必要列が必要です")
    return metadata, rows


def plot(path: Path, output: Path) -> None:
    import matplotlib.pyplot as plt

    metadata, rows = read_log(path)
    if metadata["logSchemaVersion"] == "10":
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot([0.0] + [float(row["x"]) for row in rows],
                [0.0] + [float(row["y"]) for row in rows], label="経路候補・ジャイロXY")
        ax.plot([0.0] + [float(row["x_fused_mm"]) for row in rows],
                [0.0] + [float(row["y_fused_mm"]) for row in rows],
                linestyle="--", alpha=0.55, label="左右エンコーダ差の診断XY")
        ax.scatter([0.0], [0.0], marker="x", color="black", zorder=3)
        if int(metadata.get("goalMarkerOnset_p", "0")) > 0 and \
                float(metadata.get("goalMarkerS_mm", "0")) > 0.0:
            ax.scatter([float(metadata["goalMarkerXRaw_mm"])],
                       [float(metadata["goalMarkerYRaw_mm"])],
                       marker="x", color="red", zorder=3, label="ゴール検出点（検証のみ）")
        ax.set(xlabel="X [mm]", ylabel="Y [mm]",
               title=f"{path.stem}: ジャイロ経路候補 / closureReason={metadata.get('closureReason')}")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=180, bbox_inches="tight")
        plt.close(fig)
        return
    if metadata.get("closureValid") != "1":
        raise ValueError(f"経路は無効です: closureReason={metadata.get('closureReason')}")
    goal_p = int(metadata["goalMarkerOnset_p"])
    goal_y = float(metadata["goalMarkerYRaw_mm"])
    schema9 = metadata["logSchemaVersion"] == "9"
    goal_x = float(metadata["goalMarkerXRaw_mm"]) if schema9 else 0.0
    x_field = "x_fused_mm" if schema9 else "x_closed_mm"
    y_field = "y_fused_mm" if schema9 else "y_closed_mm"
    before_goal = [row for row in rows if int(row["encTotalOptimal"]) <= goal_p]
    after_goal = [row for row in rows if int(row["encTotalOptimal"]) > goal_p]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot([0.0] + [float(row["x"]) for row in rows],
            [0.0] + [float(row["y"]) for row in rows], label="未補正XY", alpha=0.6)
    ax.plot([0.0] + [float(row[x_field]) for row in before_goal] + [goal_x],
            [0.0] + [float(row[y_field]) for row in before_goal] + [goal_y],
            label="融合XY（無補正）" if schema9 else "補正XY（マーカーまで）")
    if after_goal:
        ax.plot([goal_x] + [float(row[x_field]) for row in after_goal],
                [goal_y] + [float(row[y_field]) for row in after_goal],
                linestyle="--", label="ゴール後の停止区間")
    ax.scatter([0.0, goal_x], [0.0, goal_y], marker="x", color="black", zorder=3)
    ax.set(xlabel="X [mm]", ylabel="Y [mm]",
           title=f"{path.stem}: {'無補正融合XY' if schema9 else '右マーカー基準のX閉路'}")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    plot(args.log, args.output or Path("analysis") / f"{args.log.stem}_closed_xy.png")
