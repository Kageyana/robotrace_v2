"""分類計画の検証と適用。既定はdry-run。--applyのみ移動する。"""
import argparse
import csv
import hashlib
import json
import os
import re
from pathlib import Path

from classify_logs import DEFAULT_ROOT, DEFAULT_OUT


def move_without_overwrite(source, destination):
    """Windowsの排他的renameを使う。POSIXでも既存宛先を上書きしない。"""
    if os.name == "nt":
        source.rename(destination)
    else:
        destination.hardlink_to(source)
        source.unlink()


def validate_plan(root, plan):
    """全件検証してから移動を始める。対象変更・上書き・参照ログ移動を禁止。"""
    manifest = json.loads(plan.with_suffix(".json").read_text(encoding="utf-8"))
    if Path(manifest["root"]).resolve() != root:
        raise ValueError("plan root mismatch")
    if hashlib.sha256(plan.read_bytes()).hexdigest() != manifest["planSha256"]:
        raise ValueError("plan content changed; regenerate classification")
    sources, destinations, actions = set(), set(), []
    with plan.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["status"] != "AUTO" or not re.fullmatch(r"\d+\.csv", row["source"]):
                raise ValueError(f"not an AUTO root CSV: {row['source']}")
            if not re.fullmatch(r"course_\d{3}/\d+\.csv", row["destination"]):
                raise ValueError("invalid destination")
            source = (root / row["source"]).resolve()
            destination = (root / row["destination"]).resolve()
            if source.parent != root or root not in destination.parents:
                raise ValueError("path escapes root or source is reference log")
            if source.name != destination.name:
                raise ValueError("file name mismatch")
            if source in sources or destination in destinations:
                raise ValueError("duplicate plan entry")
            if destination.exists():
                raise ValueError(f"destination exists: {destination}")
            if not source.is_file() or source.stat().st_size != int(row["sizeBytes"]):
                raise ValueError(f"source missing/changed: {source}")
            if hashlib.sha256(source.read_bytes()).hexdigest() != row["sha256"]:
                raise ValueError(f"source hash changed: {source}")
            sources.add(source)
            destinations.add(destination)
            actions.append((source, destination, row))
    return actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--plan", type=Path, default=DEFAULT_OUT / "move_plan.csv")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    actions = validate_plan(args.root.resolve(), args.plan.resolve())
    print(f"validated {len(actions)} moves; mode={'APPLY' if args.apply else 'DRY-RUN'}")
    if not args.apply:
        for source, destination, _ in actions[:10]:
            print(f"{source.name} -> {destination.parent.name}/{destination.name}")
        return
    journal = args.plan.parent / "move_journal.jsonl"
    # append-only journalで中断時の実施済み対象を追跡。
    with journal.open("a", encoding="utf-8") as stream:
        for source, destination, row in actions:
            if hashlib.sha256(source.read_bytes()).hexdigest() != row["sha256"]:
                raise ValueError(f"source changed after preflight: {source}")
            destination.parent.mkdir(exist_ok=True)
            stream.write(json.dumps({"source": str(source), "destination": str(destination), "stage": "planned"}) + "\n")
            stream.flush()
            move_without_overwrite(source, destination)
            stream.write(json.dumps({"source": str(source), "destination": str(destination), "stage": "moved"}) + "\n")
            stream.flush()
    print(f"moved {len(actions)} files; journal={journal}")


if __name__ == "__main__":
    main()
