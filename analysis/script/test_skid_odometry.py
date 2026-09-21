"""四輪スキッドステア診断の単位・符号・校正ゲートを検証する。"""

from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from analysis.script.analyze_skid_odometry import (
    LateralState, advance_pose, check_distance, diagnose_turns, replay_lateral,
    turn_sample,
)


class SkidOdometryTest(unittest.TestCase):
    def test_turn_sample_uses_cw_positive_and_pulse_per_meter(self) -> None:
        ppm = 58019.0
        dt = 0.01
        omega = 1.0  # CW rad/s
        left_mm = 10.0 + 109.0 * omega * dt / 2.0
        right_mm = 10.0 - 109.0 * omega * dt / 2.0
        tread, speed = turn_sample(left_mm * ppm / 1000.0,
                                    right_mm * ppm / 1000.0,
                                    math.degrees(omega), dt, ppm, ppm)
        self.assertAlmostEqual(tread, 109.0, places=6)
        self.assertAlmostEqual(speed, 1.0, places=6)

    def test_turn_diagnostic_accepts_explicit_provisional_side_scales(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            with path.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["logSchemaVersion=10", "optimalTrace=0", "emcStop=0",
                                 "logOverflowFinal=0", "dbgOverflowFinal=0",
                                 "encoderPulsePerMeter=58019", "closureValid=0",
                                 "logExpectedRows=1"])
                writer.writerow(["cntlog", "gyroVal_Z", "encIntervalL_p", "encIntervalR_p"])
                writer.writerow([10, 200, 700, 600])
            common = diagnose_turns(path)
            side = diagnose_turns(path, (58092.0, 57945.0))
            self.assertEqual(common["left_ppm"], 58019.0)
            self.assertEqual(side["left_ppm"], 58092.0)
            self.assertEqual(side["right_ppm"], 57945.0)
            self.assertIn("provisional", side["side_scale_source"])
            self.assertNotEqual(common["bins"][0]["tread_median_mm"],
                                side["bins"][0]["tread_median_mm"])
            with self.assertRaises(ValueError):
                diagnose_turns(path, (60000.0, 57945.0))

    def test_lateral_velocity_moves_pose_right_and_is_bounded(self) -> None:
        state = LateralState()
        residual, x, y = advance_pose(
            state, delta_forward_mm=10.0, gyro_dps=0.0,
            lateral_accel_mps2=10.0, dt_s=0.01, bias_mps2=0.0,
            time_constant_s=1.0, max_lateral_mps=0.05,
        )
        self.assertAlmostEqual(residual, 10.0)
        self.assertAlmostEqual(state.lateral_mps, 0.05)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 10.0)

    def test_centripetal_acceleration_alone_has_no_lateral_motion(self) -> None:
        state = LateralState()
        residual, _, _ = advance_pose(
            state, delta_forward_mm=10.0, gyro_dps=math.degrees(1.0),
            lateral_accel_mps2=1.0, dt_s=0.01, bias_mps2=0.0,
            time_constant_s=0.15, max_lateral_mps=0.5,
        )
        self.assertAlmostEqual(residual, 0.0, places=6)
        self.assertAlmostEqual(state.lateral_mps, 0.0, places=6)

    def test_powered_scale_requires_each_of_three_runs_within_one_percent(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "measurement.csv"
            with path.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["run_id", "actual_mm", "left_p", "right_p", "powered"])
                for run_id in range(3):
                    writer.writerow([run_id, 1000, 58019, 58019, 1])
            self.assertTrue(check_distance(path, 58019.0)["distance_scale_verified"])
            with path.open("a", encoding="utf-8", newline="") as target:
                csv.writer(target).writerow([3, 1000, 60000, 60000, 1])
            self.assertFalse(check_distance(path, 58019.0)["distance_scale_verified"])

    def test_rounded_motor_test_display_includes_one_mm_error_bound(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "display.csv"
            with path.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["run_id", "actual_mm", "left_start_mm", "left_end_mm",
                                 "right_start_mm", "right_end_mm", "powered"])
                writer.writerow([1, 1000, 20, 1020, 17, 1017, 1])
                writer.writerow([2, 1000, 1020, 2020, 1017, 2017, 1])
                writer.writerow([3, 1000, 2020, 3028, 2017, 3017, 1])
            result = check_distance(path, 58019.0)
            self.assertTrue(result["distance_scale_verified"])
            self.assertEqual(result["trials"][0]["display_quantization_bound_mm"], 1.0)
            with path.open("a", encoding="utf-8", newline="") as target:
                csv.writer(target).writerow([4, 1000, 3028, 3038, 3017, 4017, 1])
            self.assertFalse(check_distance(path, 58019.0)["distance_scale_verified"])

    def test_display_deltas_verify_forward_but_flag_borderline_side(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "display_deltas.csv"
            with path.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["run_id", "actual_mm", "left_delta_mm",
                                 "right_delta_mm", "powered"])
                writer.writerows([
                    [1, 998, 1002, 1000, 1],
                    [2, 1002, 1010, 1005, 1],
                    [3, 1003, 1005, 996, 1],
                    [4, 998, 1001, 989, 1],
                    [5, 1011, 1011, 1002, 1],
                ])
            result = check_distance(path, 58019.0)
            self.assertTrue(result["observed_scale_within_one_percent"])
            self.assertTrue(result["distance_scale_verified"])
            self.assertFalse(result["side_scale_verified"])
            self.assertAlmostEqual(result["trials"][3]["side_worst_case_abs_error_percent"],
                                   1000.0 / 998.0)

    def test_reference_comparison_keeps_zero_slip_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "test.csv"
            reference = root / "camera.csv"
            with log.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["logSchemaVersion=6", "optimalTrace=0", "emcStop=0",
                                 "logOverflowFinal=0", "dbgOverflowFinal=0",
                                 "logExpectedRows=40"])
                writer.writerow(["cntlog", "encTotalOptimal", "gyroVal_Z",
                                 "imuLinearAccelX_mps2", "x", "y"])
                for index in range(1, 41):
                    writer.writerow([index * 10, index * 580, 0, 0, 0,
                                     index * 580 * 1000 / 53424])
            final_y = 40 * 580 * 1000 / 53424
            with reference.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["cntlog", "x_mm", "y_mm"])
                writer.writerow([0, 0, 0])
                writer.writerow([400, 0, final_y])
            result = replay_lateral(log, root / "out", reference, 0.15, 0.5)
            self.assertTrue(result["reference_validated"])
            self.assertAlmostEqual(result["baseline_position_error_p95_mm"], 0, places=6)
            self.assertAlmostEqual(result["candidate_position_error_p95_mm"], 0, places=6)


if __name__ == "__main__":
    unittest.main()
