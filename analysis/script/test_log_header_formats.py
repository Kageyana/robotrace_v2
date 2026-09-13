#!/usr/bin/env python3
"""走行ログCSVの旧・新ヘッダー互換と変換処理を検証する。"""

from __future__ import annotations

import hashlib
import os
import tempfile
import unittest
from pathlib import Path

from normalize_log_headers import inspect_log, normalized_bytes, replace_atomically
from path_log_recovery import read_csv_log


FIELDS = "cntlog,encCurrentN,gyroVal_Z,courseMarker,encTotalOptimal,ROC,x,y,"
METADATA = "fwVersion=v2-dev,optimalTrace=0.00,emcStop=0.00,"
DATA = "10,54,1.5,0,540,3000.0,0.0,10.0,\n20,55,1.0,0,1090,3000.0,0.1,20.0,\n"


class LogHeaderFormatTests(unittest.TestCase):
    def temporary_directory(self) -> tempfile.TemporaryDirectory[str]:
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


if __name__ == "__main__":
    unittest.main()
