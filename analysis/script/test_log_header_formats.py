#!/usr/bin/env python3
"""走行ログCSVの旧・新ヘッダー互換と変換処理を検証する。"""

from __future__ import annotations

import hashlib
import os
import tempfile
import unittest
from pathlib import Path

try:
    from .normalize_log_headers import inspect_log, normalized_bytes, replace_atomically
    from .path_log_recovery import build_route, read_csv_log, recover_path_columns
    from .repair_cntlog_wrap import decode_distance_pulse
except ImportError:  # ファイル単体実行との互換
    from normalize_log_headers import inspect_log, normalized_bytes, replace_atomically
    from path_log_recovery import build_route, read_csv_log, recover_path_columns
    from repair_cntlog_wrap import decode_distance_pulse


FIELDS = "cntlog,encCurrentN,gyroVal_Z,courseMarker,encTotalOptimal,ROC,x,y,"
FIELDS_V6 = "cntlog,encCurrentN,gyroVal_Z,imuLinearAccelX_mps2,imuLinearAccelY_mps2,imuLinearAccelZ_mps2,courseMarker,encTotalOptimal,ROC,x,y,"
METADATA = "fwVersion=v2-dev,logSchemaVersion=5,optimalTrace=0.00,emcStop=0.00,"
DATA = "10,54,1.5,0,540,3000.0,0.0,10.0,\n20,55,1.0,0,1090,3000.0,0.1,20.0,\n"
DATA_V6 = "10,54,1.5,0.1,0.2,0.3,0,540,3000.0,0.0,10.0,\n20,55,1.0,0.4,0.5,0.6,0,1090,3000.0,0.1,20.0,\n"


class LogHeaderFormatTests(unittest.TestCase):
    def temporary_directory(self) -> tempfile.TemporaryDirectory[str]:
        # サンドボックス内で実行できるよう、リポジトリ内へ一時領域を作る。
        return tempfile.TemporaryDirectory(dir=Path.cwd())

    def write_log(self, directory: Path, text: str, name: str = "1.csv") -> Path:
        path = directory / name
        path.write_bytes(text.encode("utf-8"))
        return path

    def test_mixed_header_is_split_without_changing_data(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(Path(temp_dir), FIELDS + METADATA + "\r\n" + DATA)
            before = inspect_log(path)
            output = normalized_bytes(before)
            path.write_bytes(output)
            after = inspect_log(path)
            self.assertEqual(after.state, "normalized")
            self.assertEqual(after.metadata, before.metadata)
            self.assertEqual(after.fields, before.fields)
            self.assertEqual(after.data_sha256, hashlib.sha256(DATA.encode("utf-8")).hexdigest())

    def test_shared_reader_accepts_mixed_and_normalized_headers(self) -> None:
        with self.temporary_directory() as temp_dir:
            directory = Path(temp_dir)
            mixed = self.write_log(directory, FIELDS + METADATA + "\n" + DATA, "1.csv")
            normalized = self.write_log(directory, METADATA + "\n" + FIELDS + "\n" + DATA, "2.csv")
            for path in (mixed, normalized):
                log = read_csv_log(path)
                self.assertEqual(log.parameters["fwVersion"], "v2-dev")
                self.assertEqual(log.fields[0], "cntlog")
                self.assertEqual(len(log.rows), 2)
                self.assertEqual(log.rows[1]["encTotalOptimal"], "1090")

    def test_shared_reader_accepts_metadata_free_header(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(Path(temp_dir), FIELDS + "\n" + DATA)
            log = read_csv_log(path)
            self.assertEqual(log.parameters, {})
            self.assertEqual(len(log.rows), 2)

    def test_normalizer_rejects_metadata_free_header(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(Path(temp_dir), FIELDS + "\n" + DATA)
            with self.assertRaisesRegex(ValueError, "メタデータがありません"):
                inspect_log(path)

    def test_normalizer_is_idempotent_for_new_header(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(Path(temp_dir), METADATA + "\n" + FIELDS + "\n" + DATA)
            layout = inspect_log(path)
            self.assertEqual(layout.state, "normalized")

    def test_atomic_replace_preserves_modified_time(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(Path(temp_dir), FIELDS + METADATA + "\n" + DATA)
            expected_mtime_ns = 1_700_000_000_000_000_000
            os.utime(path, ns=(expected_mtime_ns, expected_mtime_ns))
            replace_atomically(inspect_log(path))
            self.assertEqual(inspect_log(path).state, "normalized")
            self.assertEqual(path.stat().st_mtime_ns, expected_mtime_ns)

    def test_schema_versions_2_to_6_are_read_by_header_name(self) -> None:
        with self.temporary_directory() as temp_dir:
            directory = Path(temp_dir)
            for version in (2, 3, 4, 5, 6):
                metadata = f"logSchemaVersion={version},optimalTrace=0,emcStop=0,"
                path = self.write_log(directory, metadata + "\n" + FIELDS + "\n" + DATA,
                                      f"{version}.csv")
                log = read_csv_log(path)
                self.assertEqual(log.parameters["logSchemaVersion"], str(version))
                self.assertEqual(log.rows[0]["encTotalOptimal"], "540")

    def test_schema_version_6_reads_linear_acceleration_columns_by_name(self) -> None:
        with self.temporary_directory() as temp_dir:
            path = self.write_log(
                Path(temp_dir),
                "logSchemaVersion=6,optimalTrace=0,emcStop=0,\n" + FIELDS_V6 + "\n" + DATA_V6,
                "6.csv",
            )
            log = read_csv_log(path)
            self.assertEqual(
                log.fields[3:6],
                [
                    "imuLinearAccelX_mps2",
                    "imuLinearAccelY_mps2",
                    "imuLinearAccelZ_mps2",
                ],
            )
            self.assertEqual(log.rows[1]["imuLinearAccelY_mps2"], "0.5")

    def test_schema_versions_5_and_6_generate_the_same_recovered_path(self) -> None:
        with self.temporary_directory() as temp_dir:
            directory = Path(temp_dir)
            source_path = self.write_log(
                directory,
                "logSchemaVersion=5,optimalTrace=0,emcStop=0,\n"
                "x,y,courseMarker\n"
                "0,0,0\n0,100,0\n0,200,0\n",
                "1.csv",
            )
            source = read_csv_log(source_path)
            route = build_route(source, 0, 0)
            common = (
                "optimalTrace=3,emcStop=0,analysisSourceLog=1,routeSourceLog=1,"
                "shortcutRequestedLevel=0,routePointCount={route_count},"
                "routeGeometryCrc32={crc},routeControllerVersion=13,"
                "shortcutLevel={level},shortcutBuildStatus={status},"
                "shortcutCorridorCount={corridors},shortcutReduction_mm={reduction},"
                "routeShortcutSettings.maxLevel=0,"
            ).format(
                route_count=route.route_count,
                crc=route.geometry_crc32,
                level=route.applied_level,
                status=route.build_status,
                corridors=route.corridor_count,
                reduction=route.reduction_mm,
            )
            recovered: dict[int, list[dict[str, object]]] = {}
            for version in (5, 6):
                secondary_path = self.write_log(
                    directory,
                    f"logSchemaVersion={version}," + common + "\noptimalIndex\n0\n",
                    f"{version + 1}.csv",
                )
                result = recover_path_columns(read_csv_log(secondary_path))
                self.assertEqual(result.status, "restored")
                recovered[version] = result.row_values
            self.assertEqual(recovered[5], recovered[6])

    def test_distance_pulse_decode_keeps_old_u32_compatibility(self) -> None:
        self.assertEqual(decode_distance_pulse("4294967295", 4), -1)
        self.assertEqual(decode_distance_pulse("-1", 5), -1)


if __name__ == "__main__":
    unittest.main()
