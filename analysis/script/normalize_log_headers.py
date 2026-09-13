#!/usr/bin/env python3
"""数値名の走行ログCSVをメタデータ、列名、データの3段構成へ統一する。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path


NUMERIC_LOG_NAME = re.compile(r"^(\d+)\.csv$")
UTF8_BOM = b"\xef\xbb\xbf"


@dataclass(frozen=True)
class PhysicalLine:
    content: bytes
    separator: bytes
    next_offset: int


@dataclass(frozen=True)
class LogLayout:
    path: Path
    state: str
    metadata: tuple[str, ...]
    fields: tuple[str, ...]
    data_offset: int
    data_sha256: str
    source_sha256: str
    first_separator: bytes
    has_bom: bool


def read_physical_line(data: bytes, offset: int) -> PhysicalLine:
    newline = data.find(b"\n", offset)
    if newline < 0:
        raise ValueError("ヘッダー行の改行がありません")
    content_end = newline
    if content_end > offset and data[content_end - 1] == 0x0D:
        content_end -= 1
    return PhysicalLine(data[offset:content_end], data[content_end:newline + 1], newline + 1)


def parse_csv_line(raw: bytes, *, strip_bom: bool = False) -> tuple[str, ...]:
    if strip_bom and raw.startswith(UTF8_BOM):
        raw = raw[len(UTF8_BOM):]
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("ヘッダーがUTF-8ではありません") from exc
    rows = list(csv.reader([text]))
    if len(rows) != 1:
        raise ValueError("ヘッダーを1行のCSVとして解釈できません")
    return tuple(cell for cell in rows[0] if cell != "")


def split_cells(cells: tuple[str, ...]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    metadata = tuple(cell for cell in cells if "=" in cell)
    fields = tuple(cell for cell in cells if "=" not in cell)
    return metadata, fields


def validate_mixed_order(cells: tuple[str, ...]) -> None:
    metadata_started = False
    for cell in cells:
        if "=" in cell:
            metadata_started = True
        elif metadata_started:
            raise ValueError("メタデータの後ろに列名があり、安全に分離できません")


def inspect_data(path: Path, data: bytes) -> LogLayout:
    if not data:
        raise ValueError("空ファイルです")
    has_bom = data.startswith(UTF8_BOM)
    first = read_physical_line(data, 0)
    first_cells = parse_csv_line(first.content, strip_bom=True)
    first_metadata, first_fields = split_cells(first_cells)

    if first_metadata and first_fields:
        validate_mixed_order(first_cells)
        state = "mixed"
        metadata = first_metadata
        fields = first_fields
        data_offset = first.next_offset
    elif first_metadata:
        second = read_physical_line(data, first.next_offset)
        second_cells = parse_csv_line(second.content)
        second_metadata, second_fields = split_cells(second_cells)
        if second_metadata or not second_fields:
            raise ValueError("2行目が列名専用行ではありません")
        state = "normalized"
        metadata = first_metadata
        fields = second_fields
        data_offset = second.next_offset
    elif first_fields:
        raise ValueError("メタデータがありません")
    else:
        raise ValueError("有効なヘッダー項目がありません")

    if "cntlog" not in fields:
        raise ValueError("列名行にcntlogがありません")
    metadata_names = [cell.split("=", 1)[0].strip() for cell in metadata]
    if len(metadata_names) != len(set(metadata_names)):
        raise ValueError("重複するメタデータ項目があります")
    if len(fields) != len(set(fields)):
        raise ValueError("重複する列名があります")
    if data_offset >= len(data):
        raise ValueError("データ行がありません")
    first_data_end = data.find(b"\n", data_offset)
    if first_data_end < 0:
        first_data_end = len(data)
    first_data = data[data_offset:first_data_end].rstrip(b"\r")
    try:
        first_data_cells = next(csv.reader([first_data.decode("utf-8")]))
    except (UnicodeDecodeError, csv.Error) as exc:
        raise ValueError("先頭データ行をCSVとして解釈できません") from exc
    if len(first_data_cells) < len(fields):
        raise ValueError("先頭データ行の列数が列名より少ないです")

    return LogLayout(
        path=path,
        state=state,
        metadata=metadata,
        fields=fields,
        data_offset=data_offset,
        data_sha256=hashlib.sha256(data[data_offset:]).hexdigest(),
        source_sha256=hashlib.sha256(data).hexdigest(),
        first_separator=first.separator,
        has_bom=has_bom,
    )


def inspect_log(path: Path) -> LogLayout:
    return inspect_data(path, path.read_bytes())


def encode_header(cells: tuple[str, ...], separator: bytes) -> bytes:
    # ファームウェアのCSV契約ではヘッダー項目内にカンマを許可しない。
    if any("," in cell or "\r" in cell or "\n" in cell for cell in cells):
        raise ValueError("ヘッダー項目に未対応のカンマまたは改行があります")
    return ",".join(cells).encode("utf-8") + b"," + separator


def normalized_bytes(layout: LogLayout) -> bytes:
    source = layout.path.read_bytes()
    if hashlib.sha256(source).hexdigest() != layout.source_sha256:
        raise RuntimeError(f"検査後にファイルが変更されました: {layout.path.name}")
    prefix = UTF8_BOM if layout.has_bom else b""
    result = (
        prefix
        + encode_header(layout.metadata, layout.first_separator)
        + encode_header(layout.fields, layout.first_separator)
        + source[layout.data_offset:]
    )
    converted = inspect_data(layout.path, result)
    if converted.state != "normalized":
        raise RuntimeError(f"変換結果が新形式ではありません: {layout.path.name}")
    if converted.metadata != layout.metadata or converted.fields != layout.fields:
        raise RuntimeError(f"ヘッダー項目が変化しました: {layout.path.name}")
    if converted.data_sha256 != layout.data_sha256:
        raise RuntimeError(f"データ部分が変化しました: {layout.path.name}")
    return result


def replace_atomically(layout: LogLayout) -> None:
    output = normalized_bytes(layout)
    source_stat = layout.path.stat()
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=layout.path.parent, prefix=f".{layout.path.name}.",
            suffix=".tmp", delete=False
        ) as destination:
            temp_path = Path(destination.name)
            destination.write(output)
            destination.flush()
            os.fsync(destination.fileno())
        shutil.copystat(layout.path, temp_path)
        os.replace(temp_path, layout.path)
        os.utime(layout.path, ns=(source_stat.st_atime_ns, source_stat.st_mtime_ns))
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def numeric_logs(log_dir: Path) -> tuple[list[Path], int]:
    numeric: list[tuple[int, Path]] = []
    excluded = 0
    for path in log_dir.glob("*.csv"):
        match = NUMERIC_LOG_NAME.fullmatch(path.name)
        if match:
            numeric.append((int(match.group(1)), path))
        else:
            excluded += 1
    numeric.sort(key=lambda item: item[0])
    return [path for _, path in numeric], excluded


def validate_expected_range(paths: list[Path], expected_min: int | None, expected_max: int | None) -> None:
    if (expected_min is None) != (expected_max is None):
        raise ValueError("--expected-minと--expected-maxは同時に指定してください")
    if expected_min is None:
        return
    actual = [int(path.stem) for path in paths]
    expected = list(range(expected_min, expected_max + 1))
    if actual != expected:
        missing = sorted(set(expected) - set(actual))
        unexpected = sorted(set(actual) - set(expected))
        raise ValueError(f"対象番号が一致しません: missing={missing}, unexpected={unexpected}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--expected-min", type=int)
    parser.add_argument("--expected-max", type=int)
    parser.add_argument("--apply", action="store_true", help="検査後に旧混在形式を原子的に置換する")
    args = parser.parse_args()

    if not args.log_dir.is_dir():
        parser.error(f"ログフォルダがありません: {args.log_dir}")
    paths, excluded = numeric_logs(args.log_dir)
    validate_expected_range(paths, args.expected_min, args.expected_max)

    layouts: list[LogLayout] = []
    errors: list[str] = []
    for path in paths:
        try:
            layouts.append(inspect_log(path))
        except (OSError, ValueError) as exc:
            errors.append(f"{path.name}: {exc}")
    mixed = [layout for layout in layouts if layout.state == "mixed"]
    normalized = [layout for layout in layouts if layout.state == "normalized"]
    print(f"targets={len(paths)} mixed={len(mixed)} normalized={len(normalized)} excluded_csv={excluded} invalid={len(errors)}")
    for error in errors:
        print(f"ERROR {error}")
    if errors:
        return 1
    if not args.apply:
        return 0

    for index, layout in enumerate(mixed, start=1):
        replace_atomically(layout)
        if index % 25 == 0 or index == len(mixed):
            print(f"converted={index}/{len(mixed)}")

    post_layouts = [inspect_log(path) for path in paths]
    remaining = [layout.path.name for layout in post_layouts if layout.state != "normalized"]
    if remaining:
        print(f"ERROR 新形式でないファイルが残っています: {remaining}")
        return 1
    print(f"verified={len(post_layouts)} changed={len(mixed)} remaining=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
