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
    @staticmethod
    def _write_schema10_turn_log(
        path: Path,
        *,
        gyro_bias_dps: float = 0.0,
        cw_tread_mm: float = 120.0,
        ccw_tread_mm: float = 120.0,
        turn_rates: tuple[float, ...] = (0.25, 0.5, 0.9),
        speed_profiles: tuple[tuple[float, tuple[int, ...]], ...] | None = None,
        include_wrap: bool = True,
        trailing_empty_cell: bool = True,
    ) -> None:
        ppm = 58019.0
        rows = []
        if include_wrap:
            # 25 msごとの約10 mm直進を記録し、16 bit cntlogを実際に一度折り返す。
            for _ in range(2622):
                dt = 25
                distance = 0.4 * dt
                pulses = round(distance * ppm / 1000.0)
                rows.append([dt, gyro_bias_dps, pulses, pulses])
        speed_dt = speed_profiles or (
            (0.35, (20, 25, 30)), (0.8, (8, 10, 12)),
            (1.5, (5, 6, 7)), (3.0, (3, 4, 5)),
        )
        for speed, intervals in speed_dt:
            for direction in (1.0, -1.0):
                for omega in turn_rates:
                    for index in range(16):
                        dt_ms = intervals[index % len(intervals)]
                        dt_s = dt_ms * 0.001
                        true_omega = direction * omega
                        mean_distance_mm = speed * dt_s * 1000.0
                        tread_mm = cw_tread_mm if direction > 0 else ccw_tread_mm
                        differential_mm = tread_mm * true_omega * dt_s
                        left_p = round((mean_distance_mm + differential_mm / 2.0) * ppm / 1000.0)
                        right_p = round((mean_distance_mm - differential_mm / 2.0) * ppm / 1000.0)
                        gyro = math.degrees(true_omega) + gyro_bias_dps
                        rows.append([dt_ms, gyro, left_p, right_p])
                    # 旋回セグメントを分ける直進行。gyroには一定バイアスだけが残る。
                    dt_ms = intervals[0]
                    distance = speed * dt_ms
                    pulses = round(distance * ppm / 1000.0)
                    rows.append([dt_ms, gyro_bias_dps, pulses, pulses])

        timestamp = 0
        def with_optional_trailing_empty(cells: list[object]) -> list[object]:
            return cells + ([""] if trailing_empty_cell else [])

        with path.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(with_optional_trailing_empty([
                "logSchemaVersion=10", "optimalTrace=0", "emcStop=0",
                "logOverflowFinal=0", "dbgOverflowFinal=0", "encoderPulsePerMeter=58019",
                "distanceScaleVerified=1", "closureValid=1", "closureReason=0",
                "imuCalibrationValid=1", "imuCalibrationSamples=100",
                "imuCalibrationReadErrors=0", "gyroSampleFault=0", "encoderIntervalFault=0",
                f"logExpectedRows={len(rows)}", "headingCalibration.enabled=0",
                "headingCalibration.pulsePerMeterL=58092",
                "headingCalibration.pulsePerMeterR=57945",
                "headingCalibration.effectiveTread_mm=106.02",
            ]))
            writer.writerow(with_optional_trailing_empty(
                ["cntlog", "gyroVal_Z", "encIntervalL_p", "encIntervalR_p"]
            ))
            for dt_ms, gyro, left_p, right_p in rows:
                timestamp = (timestamp + dt_ms) & 0xFFFF
                writer.writerow(with_optional_trailing_empty([timestamp, gyro, left_p, right_p]))

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

        ccw_tread, _ = turn_sample(right_mm * ppm / 1000.0,
                                   left_mm * ppm / 1000.0,
                                   -math.degrees(omega), dt, ppm, ppm)
        self.assertAlmostEqual(ccw_tread, 109.0, places=6)

    def test_turn_diagnostic_accepts_explicit_provisional_side_scales(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            self._write_schema10_turn_log(path)
            common = diagnose_turns(path)
            side = diagnose_turns(path, (58092.0, 57945.0))
            self.assertEqual(common["left_ppm"], 58019.0)
            self.assertEqual(side["left_ppm"], 58092.0)
            self.assertEqual(side["right_ppm"], 57945.0)
            self.assertIn("provisional", side["side_scale_source"])
            self.assertNotEqual(common["bins"][0]["tread_median_mm"],
                                side["bins"][0]["tread_median_mm"])
            self.assertEqual(common["cntlog_interval_ms"]["max"], 30)
            self.assertEqual(common["cntlog_interval_ms"]["u16_wraps"], 1)
            self.assertEqual(common["samples_total"], common["input_checks"]["logExpectedRows"] and
                             int(common["input_checks"]["logExpectedRows"]))
            self.assertEqual(len({item["turn"] for item in common["bins"]}), 2)
            low_cw = next(item for item in common["bins"]
                          if item["speed_mps"] == "0.3-1" and item["turn"] == "CW")
            self.assertAlmostEqual(low_cw["tread_fit_mm"], 120.0, delta=2.0)
            self.assertAlmostEqual(low_cw["icr_candidate_left_mm"], -60.0, delta=1.0)
            self.assertAlmostEqual(low_cw["icr_candidate_right_mm"], 60.0, delta=1.0)
            self.assertEqual(common["time_proportional_offset_by_speed_band"]["0.3-1"]["segments"], 12)
            self.assertTrue(all(
                "gyro_bias" not in key and "bias_adjusted" not in key and key != "identifiable"
                for key in common.keys()
            ))
            self.assertTrue(all(
                "gyro_bias" not in key and "bias_adjusted" not in key and key != "identifiable"
                for item in common["bins"]
                for key in item.keys()
            ))
            self.assertAlmostEqual(
                low_cw["difference_from_heading_calibration_effective_tread_mm"],
                low_cw["tread_fit_mm"] - 106.02,
            )
            path.write_text(path.read_text(encoding="utf-8").replace(
                "headingCalibration.enabled=0", "headingCalibration.enabled=1", 1),
                encoding="utf-8")
            enabled = diagnose_turns(path)
            self.assertEqual(enabled["left_ppm"], 58092.0)
            self.assertEqual(enabled["right_ppm"], 57945.0)
            self.assertIn("existing enabled", enabled["side_scale_source"])
            with self.assertRaises(ValueError):
                diagnose_turns(path, (60000.0, 57945.0))

    def test_different_cw_ccw_tread_is_not_interpreted_as_gyro_bias(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            self._write_schema10_turn_log(
                path, gyro_bias_dps=0.0, cw_tread_mm=128.0, ccw_tread_mm=112.0,
                speed_profiles=((0.8, (8, 10, 12)),),
                include_wrap=False,
            )
            result = diagnose_turns(path)
            cw = next(item for item in result["bins"]
                      if item["speed_mps"] == "0.3-1" and item["turn"] == "CW")
            ccw = next(item for item in result["bins"]
                       if item["speed_mps"] == "0.3-1" and item["turn"] == "CCW")
            self.assertAlmostEqual(cw["tread_fit_mm"], 128.0, delta=2.0)
            self.assertAlmostEqual(ccw["tread_fit_mm"], 112.0, delta=2.0)
            self.assertAlmostEqual(cw["gyro_minus_encoder_rate_mean_dps"], 0.0, delta=0.5)
            self.assertAlmostEqual(ccw["gyro_minus_encoder_rate_mean_dps"], 0.0, delta=0.5)
            self.assertAlmostEqual(cw["icr_candidate_left_mm"], -cw["tread_fit_mm"] / 2.0)
            self.assertAlmostEqual(cw["icr_candidate_right_mm"], cw["tread_fit_mm"] / 2.0)
            self.assertAlmostEqual(ccw["icr_candidate_left_mm"], -ccw["tread_fit_mm"] / 2.0)
            self.assertAlmostEqual(ccw["icr_candidate_right_mm"], ccw["tread_fit_mm"] / 2.0)
            self.assertEqual(result["time_proportional_offset_by_speed_band"]["0.3-1"]["segments"], 6)
            forbidden_terms = ("gyro_bias", "bias_adjusted", "identifiable")
            all_keys = [key for entry in result["bins"] for key in entry]
            all_keys.extend(result.keys())
            self.assertFalse(any(term in key for key in all_keys for term in forbidden_terms))

    def test_time_proportional_offset_is_omitted_with_only_two_turn_segments(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "two_turns.csv"
            self._write_schema10_turn_log(
                path, turn_rates=(0.5,), speed_profiles=((0.8, (8, 10, 12)),),
                include_wrap=False, trailing_empty_cell=False,
            )
            result = diagnose_turns(path)
            self.assertEqual(result["segments_used"], 2)
            self.assertEqual(result["time_proportional_offset_by_speed_band"], {})
            self.assertEqual(len(result["bins"]), 2)

    def test_turn_diagnostic_rejects_faults_missing_rows_and_invalid_primary_runs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            self._write_schema10_turn_log(path, include_wrap=False)
            original = path.read_text(encoding="utf-8")
            for metadata, replacement, message in (
                ("gyroSampleFault=0", "gyroSampleFault=1", "gyroSampleFault"),
                ("encoderIntervalFault=0", "encoderIntervalFault=1", "encoderIntervalFault"),
                ("imuCalibrationValid=1", "imuCalibrationValid=0", "imuCalibrationValid"),
                ("closureValid=1", "closureValid=0", "closureValid"),
                ("emcStop=0", "emcStop=1", "emcStop"),
            ):
                with self.subTest(message=message):
                    path.write_text(original.replace(metadata, replacement, 1), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, message):
                        diagnose_turns(path)
            path.write_text(original, encoding="utf-8")
            lines = original.splitlines()
            path.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "expected"):
                diagnose_turns(path)

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

    def test_distance_diagnostic_retains_conditions_and_display_rounding(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "measurement.csv"
            with path.open("w", encoding="utf-8", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["run_id", "actual_mm", "left_delta_mm", "right_delta_mm",
                                 "powered", "duty", "conditions"])
                writer.writerows([
                    [1, 1000, 1000, 1000, 1, 100, "室内直進"],
                    [2, 1000, 1000, 1000, 1, 100, "室内直進"],
                    [3, 1000, 1000, 1000, 1, 100, "室内直進"],
                ])
            result = check_distance(path, 58019.0)
            self.assertEqual(result["measurement_conditions"][0],
                             {"duty": "100", "conditions": "室内直進"})
            self.assertEqual(result["trials"][0]["display_quantization_bound_mm"], 1.0)
            self.assertAlmostEqual(result["trials"][0]["side_ppm_display_rounding_bound"], 58.019)

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
