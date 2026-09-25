"""独立XYを使う横ICR学習・固定検証の合成回帰テスト。"""

from __future__ import annotations

import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from analysis.script.analyze_camera_xy import (
    _read_reference, _turn_windows, evaluate_baseline, evaluate_k_lat_for_runs,
    fit_k_lat_for_runs,
)
from analysis.script.camera_xy_reference import _read_log, _wrap_degrees
from analysis.script.robotrace_units import CURRENT_PULSE_METER


class CameraXYAnalysisTests(unittest.TestCase):
    def test_turn_windows_ignore_camera_frames_after_log_end(self) -> None:
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            run = self._write_run(root, "trailing", 35.0)
            with Path(run["reference_csv"]).open("a", encoding="utf-8", newline="") as target:
                csv.writer(target).writerow([4010, 4010, 10000, 10000, 100, 1])
            timeline = _read_log(Path(run["log"]))
            reference, accuracy, _ = _read_reference(
                Path(run["reference_csv"]), Path(run["diagnostics_json"]))
            windows, _ = _turn_windows(reference, timeline, accuracy, 10.0)
            self.assertTrue(all(window["last_time_ms"] <= timeline.time_ms[-1]
                                for window in windows))

    def test_registered_camera_accuracy_uses_verified_residual(self) -> None:
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            run = self._write_run(root, "registered", 35.0)
            diagnostics_path = Path(run["diagnostics_json"])
            diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
            diagnostics["calibration"]["check_max_mm"] = 3.6
            diagnostics["tracking"].update({
                "camera_motion_mode": "register",
                "camera_motion_p95_mm": 31.7,
                "registration_error_max_mm": 2.96,
            })
            diagnostics_path.write_text(json.dumps(diagnostics), encoding="utf-8")
            _, accuracy, _ = _read_reference(Path(run["reference_csv"]), diagnostics_path)
            self.assertAlmostEqual(accuracy, 3.6)

    def test_five_train_and_five_held_out_runs_recover_k_lat(self) -> None:
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            training = [self._write_run(root, f"train{i}", 35.0) for i in range(5)]
            validation = [self._write_run(root, f"validation{i}", 35.0) for i in range(5)]

            baseline = evaluate_baseline(
                Path(training[0]["log"]), Path(training[0]["reference_csv"]),
                Path(training[0]["diagnostics_json"]))
            self.assertIn("current_pose_position_error_p95_mm", baseline)
            self.assertIn("gyro_heading_error_p95_deg", baseline)
            self.assertNotIn("candidate_pose_position_error_p95_mm", baseline)

            model = fit_k_lat_for_runs(training)
            self.assertEqual(model["training_run_ids"], [f"train{i}" for i in range(5)])
            coefficient = model["coefficients"]["0.3-1"]
            self.assertAlmostEqual(coefficient["CW"]["k_lat_mm_per_rad"], 35.0, delta=1.0)
            self.assertAlmostEqual(coefficient["CCW"]["k_lat_mm_per_rad"], 35.0, delta=1.0)
            self.assertGreaterEqual(coefficient["CW"]["segments"], 10)
            self.assertGreaterEqual(coefficient["CCW"]["segments"], 10)

            result = evaluate_k_lat_for_runs(validation, model)
            self.assertEqual(len(result["runs"]), 5)
            self.assertIn("current_pose_end_error_stdev_mm", result["run_to_run_variation"])
            self.assertIn("candidate_pose_end_error_stdev_mm", result["run_to_run_variation"])
            for run in result["runs"]:
                self.assertLess(run["candidate_pose_position_error_p95_mm"],
                                run["current_pose_position_error_p95_mm"] * 0.2)
                self.assertLess(run["gyro_heading_error_p95_deg"], 0.1)

            with self.assertRaisesRegex(ValueError, "exactly five"):
                fit_k_lat_for_runs(training[:4])
            with self.assertRaisesRegex(ValueError, "overlap"):
                evaluate_k_lat_for_runs(training, model)

    @classmethod
    def _write_run(cls, root: Path, run_id: str, true_k_lat: float) -> dict:
        log_path = root / f"{run_id}.csv"
        reference_path = root / f"{run_id}_reference.csv"
        diagnostics_path = root / f"{run_id}_diagnostics.json"
        pulse_per_ms = CURRENT_PULSE_METER * 0.8 / 1000.0
        metadata = {
            "logSchemaVersion": 10, "optimalTrace": 0, "closureValid": 1,
            "closureReason": 0, "emcStop": 0, "logOverflowFinal": 0,
            "dbgOverflowFinal": 0, "distanceScaleVerified": 1,
            "encoderPulsePerMeter": int(CURRENT_PULSE_METER), "logRecordSizeBytes": 36,
            "imuCalibrationValid": 1, "imuCalibrationSamples": 100,
            "imuCalibrationReadErrors": 0, "gyroSampleFault": 0,
            "encoderIntervalFault": 0, "timing.startResetMeasured": 1,
            "timing.startResetDelay_ms": 20, "goalMarkerOnsetValid": 1,
            "goalMarkerOnset_p": round(4000 * pulse_per_ms), "logExpectedRows": 400,
        }
        log_rows = []
        reference_rows = []
        heading = 0.0
        true_x = true_y = 0.0
        odom_x = odom_y = 0.0
        previous_pulse = 0
        pulse_count = 0
        for time_ms in range(10, 4001, 10):
            interval_index = time_ms // 10
            # Alternate 2 s CW and CCW arcs to excite both signs independently.
            turn_sign = 1.0 if (time_ms // 2000) % 2 == 0 else -1.0
            omega_dps = 100.0 * turn_sign
            dtheta = math.radians(omega_dps) * 0.01
            pulse_count = round(time_ms * pulse_per_ms)
            ds_mm = (pulse_count - previous_pulse) * 1000.0 / CURRENT_PULSE_METER
            mid = heading + dtheta * 0.5
            dlat = true_k_lat * dtheta
            true_x += ds_mm * math.sin(mid) + dlat * math.cos(mid)
            true_y += ds_mm * math.cos(mid) - dlat * math.sin(mid)
            odom_x += ds_mm * math.sin(mid)
            odom_y += ds_mm * math.cos(mid)
            heading += dtheta
            log_rows.append([time_ms, pulse_count, omega_dps, odom_x, odom_y])
            reference_rows.append([time_ms, true_x, true_y,
                                   _wrap_degrees(math.degrees(heading)), 1])
            previous_pulse = pulse_count

        metadata["goalMarkerOnset_p"] = pulse_count
        with log_path.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow([f"{key}={value}" for key, value in metadata.items()])
            writer.writerow(["cntlog", "encTotalOptimal", "gyroVal_Z", "x", "y"])
            writer.writerows(log_rows)
        with reference_path.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(["time_ms", "video_time_ms", "x_mm", "y_mm",
                             "heading_cw_deg", "track_valid"])
            writer.writerows([[t, t, x, y, h, valid] for t, x, y, h, valid in reference_rows])
        diagnostics_path.write_text(json.dumps({
            "reference_valid": True,
            "calibration": {"check_rms_mm": 0.5},
            "tracking": {"camera_motion_p95_mm": 0.2},
            "video": {"nominal_frame_period_ms": 10.0},
        }), encoding="utf-8")
        return {"run_id": run_id, "log": str(log_path),
                "reference_csv": str(reference_path),
                "diagnostics_json": str(diagnostics_path)}


if __name__ == "__main__":
    unittest.main()
