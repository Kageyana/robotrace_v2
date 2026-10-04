"""分類のデータ欠落・逆方向・参照除外・移動安全条件を検証する。"""
import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from classify_logs import parse_csv, completion, compare, reverse, auto_eligible, classify, register_clusters
from apply_classification import validate_plan, move_without_overwrite


class ClassificationTests(unittest.TestCase):
    def parse(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "1.csv"
            path.write_text(content, encoding="utf-8")
            return parse_csv(path)

    def test_old_new_and_missing(self):
        for header in ("cntlog,encTotalOptimal,gyroVal_Z,emcStop=0\n",
                       "emcStop=0\ncntlog,encTotalOptimal,gyroVal_Z\n"):
            log = self.parse(header + "1,100,5,\n2,200,,\n")
            self.assertEqual(log["rowCount"], 2)
            self.assertEqual(log["badRows"], 1)
            self.assertIsNone(log["arrays"]["gyroVal_Z"][-1])
            self.assertEqual(log["metadata"]["emcStop"], "0")
            self.assertEqual(completion(log)[0], "ABNORMAL")

    def test_truncated_and_nul(self):
        for data in ("1,100,5\n2,200\n", "1,100,5\n2,200,\0\n"):
            log = self.parse("cntlog,encTotalOptimal,gyroVal_Z,emcStop=0\n" + data)
            self.assertEqual(completion(log)[0], "ABNORMAL")

    def test_wrap_and_closure_exceptions(self):
        for metadata in ("emcStop=0,closureReason=8,closureValid=0",
                         "emcStop=0,optimalTrace=4,closureReason=1,closureValid=0"):
            log = self.parse(metadata + "\ncntlog,encTotalOptimal\n65530,100\n5,200\n")
            self.assertEqual(completion(log)[0], "NORMAL_CANDIDATE")
            self.assertEqual(log["timeWraps"], 1)
        log = self.parse("cntlog,encTotalOptimal\n1,100\n2,200\n")
        self.assertEqual(completion(log)[0], "PENDING")

    def test_corrected_distance_backsteps(self):
        for mode, expected in ((0, "ABNORMAL"), (2, "NORMAL_CANDIDATE")):
            log = self.parse(f"emcStop=0,optimalTrace={mode}\ncntlog,encTotalOptimal\n1,100\n2,200\n3,180\n4,1000\n")
            self.assertEqual(completion(log)[0], expected)
            self.assertEqual(log["distanceBacksteps"], 1)
            self.assertEqual(log["distanceMaxCorrectionPulse"], 20)

    def test_reverse_and_missing_scores(self):
        values = np.sin(np.linspace(0, 7, 512)).tolist()
        f = {"gyro": values, "rawGyro": values, "roc": values,
             "turn": np.sign(values).tolist(), "events": [[.1, 1], [.4, 3], [.8, 2]],
             "distance": 10000, "markerColumn": "courseMarker"}
        score, direction, parts = compare(reverse(f), f)
        self.assertAlmostEqual(score, 1)
        self.assertEqual(direction, "reverse")
        missing = dict(f, events=None, roc=None)
        result = compare(missing, f)
        self.assertIsNone(result[2]["markerSequence"])
        self.assertIsNone(result[2]["roc"])
        self.assertFalse(auto_eligible((result[0], "course_001", result[1], result[2], None), .2))
        missing_event = dict(f, events=f["events"][:-1])
        partial = compare(missing_event, f)
        self.assertGreater(partial[2]["markerPosition"], .5)
        self.assertLess(partial[2]["markerSequence"], 1)

    def test_parent_inheritance_and_short_run(self):
        def log(n, mode, parent, distance, folder="."):
            item = self.parse(f"emcStop=0,optimalTrace={mode},routeSourceLog={parent}\ncntlog,encTotalOptimal\n1,10\n2,{distance}\n")
            item["completionStatus"], item["completionReasons"] = completion(item)
            item.update(logNumber=n, feature=None, sourceFolder=folder, relativePath=f"{folder}/{n}.csv")
            return item
        parent = log(10, 0, 0, 10000, "course_001")
        good = log(11, 4, 10, 9990)
        short = log(12, 4, 10, 3000)
        results = classify([parent, good, short], {})
        self.assertEqual(results[1]["status"], "AUTO")
        self.assertEqual(results[1]["classifiedCourse"], "course_001")
        self.assertEqual(results[2]["status"], "REVIEW")
        conflicting = log(10, 0, 0, 10000, "course_002")
        self.assertEqual(classify([parent, conflicting, good], {})[2]["status"], "REVIEW")

    def test_cluster_registration_requires_exact_members(self):
        cluster = [({"logNumber": 1, "relativePath": "1.csv"}, {"status": "UNCLASSIFIED"})]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.csv"
            row = {"cluster": "unknown_cluster_001", "representativeLog": "1", "members": '["1.csv"]', "course": "course_006"}
            def save():
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=row)
                    writer.writeheader()
                    writer.writerow(row)
            save()
            self.assertEqual(register_clusters(path, [cluster]), {"course_006"})
            self.assertEqual(cluster[0][1]["status"], "AUTO")
            row["members"] = '["2.csv"]'
            save()
            with self.assertRaisesRegex(ValueError, "members changed"):
                register_clusters(path, [cluster])

    def test_move_plan_safety(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            source = root / "1.csv"
            source.write_bytes(b"data")
            plan = root / "move_plan.csv"
            row = {"source": "1.csv", "destination": "course_001/1.csv", "status": "AUTO",
                   "sizeBytes": "4", "sha256": hashlib.sha256(b"data").hexdigest()}
            def save():
                with plan.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=row)
                    writer.writeheader()
                    writer.writerow(row)
                plan.with_suffix(".json").write_text(json.dumps({"root": str(root), "planSha256": hashlib.sha256(plan.read_bytes()).hexdigest()}))
            save()
            self.assertEqual(len(validate_plan(root, plan)), 1)
            self.assertTrue(source.exists())
            source.write_bytes(b"edit")
            with self.assertRaisesRegex(ValueError, "hash changed"):
                validate_plan(root, plan)
            source.write_bytes(b"data")
            row["source"] = "course_002/1.csv"
            save()
            with self.assertRaises(ValueError):
                validate_plan(root, plan)
            row["source"] = "1.csv"
            row["destination"] = "../course_001/1.csv"
            save()
            with self.assertRaises(ValueError):
                validate_plan(root, plan)

    def test_move_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.csv"
            destination = Path(directory) / "destination.csv"
            source.write_bytes(b"source")
            destination.write_bytes(b"existing")
            with self.assertRaises(FileExistsError):
                move_without_overwrite(source, destination)
            self.assertEqual(source.read_bytes(), b"source")
            self.assertEqual(destination.read_bytes(), b"existing")
            destination.unlink()
            move_without_overwrite(source, destination)
            self.assertFalse(source.exists())
            self.assertEqual(destination.read_bytes(), b"source")


if __name__ == "__main__":
    unittest.main()
