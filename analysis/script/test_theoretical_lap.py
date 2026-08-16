"""Tests for theoretical-lap calculation and autoStart grouping."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from analysis.script.theoretical_lap import (
    LogData,
    TheoreticalLapError,
    build_course_geometry,
    build_target_profile,
    calculate_theoretical_lap,
    read_log,
)
from analysis.script.theoretical_lap_report import (
    analyze_complete_sets,
    group_auto_start_sets,
    validate_log,
)


def make_log(
    number: int,
    rows: list[dict[str, str]],
    *,
    auto_start: int,
    optimal_trace: int,
) -> LogData:
    columns = tuple(rows[0].keys()) if rows else ()
    return LogData(
        number=number,
        path=Path(f"{number}.csv"),
        columns=columns,
        params={
            "autoStart": float(auto_start),
            "optimalTrace": float(optimal_trace),
            "emcStop": 0.0,
            "batteryVoltage_V": 8.0,
            "tgtParam.acceleF": 20.0,
            "tgtParam.acceleD": 20.0,
        },
        rows=tuple(rows),
    )


def base_rows(distances_mm: list[float]) -> list[dict[str, str]]:
    return [
        {
            "cntlog": str(index + 1),
            "encTotalOptimal": str(distance * 54.324),
            "ROC": "3000",
            "gyroVal_Z": "0",
            "x": "0",
            "y": str(distance),
        }
        for index, distance in enumerate(distances_mm)
    ]


class CourseGeometryTests(unittest.TestCase):
    def test_final_partial_segment_is_included(self) -> None:
        base = make_log(
            1,
            base_rows([10, 20, 30, 40, 50, 60, 70]),
            auto_start=1,
            optimal_trace=0,
        )
        geometry = build_course_geometry(base)

        self.assertEqual(len(geometry.segments), 2)
        self.assertAlmostEqual(geometry.distance_mm, 70.0)
        self.assertAlmostEqual(geometry.segments[0].distance_mm, 50.0)
        self.assertAlmostEqual(geometry.segments[1].distance_mm, 20.0)

    def test_constant_target_theoretical_time(self) -> None:
        base = make_log(
            1,
            base_rows([10, 20, 30, 40, 50, 60, 70]),
            auto_start=1,
            optimal_trace=0,
        )
        secondary = make_log(
            2,
            [
                {
                    "cntlog": "1",
                    "encCurrentN": "54.324",
                    "optimalIndex": "0",
                    "targetSpeed": "54.324",
                },
                {"cntlog": "80", "optimalIndex": "1", "targetSpeed": "54.324"},
            ],
            auto_start=2,
            optimal_trace=2,
        )

        result = calculate_theoretical_lap(
            build_course_geometry(base),
            secondary,
            initial_speed_mps=1.0,
        )

        self.assertAlmostEqual(result.theoretical_lap_ms, 70.0)
        self.assertAlmostEqual(result.instantaneous_theoretical_lap_ms, 70.0)
        self.assertAlmostEqual(result.accel_decel_adjustment_ms, 0.0)
        self.assertAlmostEqual(result.actual_lap_ms, 80.0)
        self.assertAlmostEqual(result.lap_gap_ms, 10.0)
        self.assertAlmostEqual(result.lap_gap_pct, 100.0 / 7.0)

    def test_acceleration_change_is_integrated_within_segment(self) -> None:
        base = make_log(
            1,
            base_rows([10, 20, 30, 40, 50, 60, 70, 80, 90, 100]),
            auto_start=1,
            optimal_trace=0,
        )
        secondary = make_log(
            2,
            [
                {
                    "cntlog": "1",
                    "encCurrentN": "54.324",
                    "optimalIndex": "0",
                    "targetSpeed": "54.324",
                },
                {"cntlog": "100", "optimalIndex": "1", "targetSpeed": "108.648"},
            ],
            auto_start=2,
            optimal_trace=2,
        )

        result = calculate_theoretical_lap(
            build_course_geometry(base),
            secondary,
            acceleration_mps2=1.0,
            deceleration_mps2=1.0,
            initial_speed_mps=1.0,
        )

        expected_ms = 50.0 + (math.sqrt(1.1) - 1.0) * 1000.0
        self.assertAlmostEqual(result.theoretical_lap_ms, expected_ms)
        self.assertEqual(result.acceleration_transition_count, 1)
        self.assertEqual(result.acceleration_unreached_count, 1)

    def test_default_initial_speed_uses_first_encoder_sample(self) -> None:
        base = make_log(
            1,
            base_rows([10, 20, 30, 40, 50, 60, 70]),
            auto_start=1,
            optimal_trace=0,
        )
        secondary = make_log(
            2,
            [
                {
                    "cntlog": "1",
                    "encCurrentN": "40.743",
                    "optimalIndex": "0",
                    "targetSpeed": "54.324",
                },
                {"cntlog": "80", "optimalIndex": "1", "targetSpeed": "54.324"},
            ],
            auto_start=2,
            optimal_trace=2,
        )

        result = calculate_theoretical_lap(build_course_geometry(base), secondary)

        self.assertAlmostEqual(result.initial_speed_mps, 0.75)
        transition_distance_m = (1.0**2 - 0.75**2) / (2.0 * 20.0)
        expected_ms = (
            (1.0 - 0.75) / 20.0
            + (0.05 - transition_distance_m) / 1.0
            + 0.02 / 1.0
        ) * 1000.0
        self.assertAlmostEqual(result.theoretical_lap_ms, expected_ms)

    def test_deceleration_change_is_integrated_within_segment(self) -> None:
        base = make_log(
            1,
            base_rows([10, 20, 30, 40, 50, 60, 70, 80, 90, 100]),
            auto_start=1,
            optimal_trace=0,
        )
        secondary = make_log(
            2,
            [
                {
                    "cntlog": "1",
                    "encCurrentN": "108.648",
                    "optimalIndex": "0",
                    "targetSpeed": "108.648",
                },
                {"cntlog": "100", "optimalIndex": "1", "targetSpeed": "54.324"},
            ],
            auto_start=2,
            optimal_trace=2,
        )

        result = calculate_theoretical_lap(
            build_course_geometry(base),
            secondary,
            acceleration_mps2=1.0,
            deceleration_mps2=1.0,
            initial_speed_mps=2.0,
        )

        exit_speed_mps = math.sqrt(3.9)
        expected_ms = 25.0 + 100.0 / (2.0 + exit_speed_mps)
        self.assertAlmostEqual(result.theoretical_lap_ms, expected_ms)
        self.assertEqual(result.deceleration_transition_count, 1)
        self.assertEqual(result.deceleration_unreached_count, 1)


class TargetProfileTests(unittest.TestCase):
    def test_mode_wins_and_tie_uses_last_value(self) -> None:
        secondary = make_log(
            2,
            [
                {"cntlog": "1", "optimalIndex": "0", "targetSpeed": "80"},
                {"cntlog": "2", "optimalIndex": "0", "targetSpeed": "100"},
                {"cntlog": "3", "optimalIndex": "1", "targetSpeed": "120"},
                {"cntlog": "4", "optimalIndex": "1", "targetSpeed": "120"},
                {"cntlog": "5", "optimalIndex": "1", "targetSpeed": "100"},
            ],
            auto_start=2,
            optimal_trace=2,
        )

        profile = build_target_profile(secondary, 2)

        self.assertEqual(profile.speeds_pulse_ms, (100.0, 120.0))

    def test_one_percent_internal_gap_is_interpolated(self) -> None:
        rows = [
            {
                "cntlog": str(index + 1),
                "optimalIndex": str(index),
                "targetSpeed": str(100 + index),
            }
            for index in range(100)
            if index != 50
        ]
        secondary = make_log(2, rows, auto_start=2, optimal_trace=2)

        profile = build_target_profile(secondary, 100)

        self.assertAlmostEqual(profile.coverage_pct, 99.0)
        self.assertEqual(profile.filled_count, 1)
        self.assertAlmostEqual(profile.speeds_pulse_ms[50], 150.0)

    def test_coverage_below_99_percent_is_rejected(self) -> None:
        rows = [
            {
                "cntlog": str(index + 1),
                "optimalIndex": str(index),
                "targetSpeed": "100",
            }
            for index in range(100)
            if index not in (49, 50)
        ]
        secondary = make_log(2, rows, auto_start=2, optimal_trace=2)

        with self.assertRaisesRegex(TheoreticalLapError, "coverage"):
            build_target_profile(secondary, 100)

    def test_edge_gap_is_rejected(self) -> None:
        secondary = make_log(
            2,
            [
                {"cntlog": str(index), "optimalIndex": str(index), "targetSpeed": "100"}
                for index in range(1, 101)
            ],
            auto_start=2,
            optimal_trace=2,
        )

        with self.assertRaisesRegex(TheoreticalLapError, "edge gap"):
            build_target_profile(secondary, 101)


class LogReadTests(unittest.TestCase):
    def test_short_csv_row_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "1.csv"
            path.write_text(
                "cntlog,targetSpeed,optimalTrace=2\n1\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(TheoreticalLapError, "expected at least 2"):
                read_log(path, 1)


class AutoStartGroupingTests(unittest.TestCase):
    def test_complete_and_incomplete_sets_are_separate(self) -> None:
        logs = [
            make_log(number, [{"cntlog": "1"}], auto_start=auto, optimal_trace=0 if auto == 1 else 2)
            for number, auto in zip(range(1, 9), (1, 2, 3, 4, 5, 1, 2, 3))
        ]

        groups = group_auto_start_sets(logs)

        self.assertEqual([[int(log.params["autoStart"]) for log in group] for group in groups], [[1, 2, 3, 4, 5], [1, 2, 3]])

    def test_incomplete_set_is_excluded(self) -> None:
        logs = [
            make_log(number, [{"cntlog": "1"}], auto_start=auto, optimal_trace=0 if auto == 1 else 2)
            for number, auto in zip(range(1, 4), (1, 2, 3))
        ]

        run_rows, course_rows, excluded = analyze_complete_sets(logs)

        self.assertEqual(run_rows, [])
        self.assertEqual(course_rows, [])
        self.assertEqual(excluded[0]["reason"], "incomplete_autoStart_set")

    def test_emergency_stop_is_invalid(self) -> None:
        log = make_log(
            2,
            [{"cntlog": "1", "encTotalOptimal": "1", "targetSpeed": "1", "optimalIndex": "0"}],
            auto_start=2,
            optimal_trace=2,
        )
        log.params["emcStop"] = 5.0

        reasons = validate_log(log, first_run=False)

        self.assertIn("emcStop=5", reasons)

    def test_missing_battery_voltage_is_invalid(self) -> None:
        log = make_log(
            2,
            [{"cntlog": "1", "encTotalOptimal": "1", "targetSpeed": "1", "optimalIndex": "180"}],
            auto_start=2,
            optimal_trace=2,
        )
        del log.params["batteryVoltage_V"]

        reasons = validate_log(log, first_run=False)

        self.assertIn("batteryVoltage_V=nan", reasons)

    def test_missing_acceleration_setting_is_invalid(self) -> None:
        log = make_log(
            2,
            [{"cntlog": "1", "encTotalOptimal": "1", "targetSpeed": "1", "optimalIndex": "180"}],
            auto_start=2,
            optimal_trace=2,
        )
        del log.params["tgtParam.acceleF"]

        reasons = validate_log(log, first_run=False)

        self.assertIn("tgtParam.acceleF=nan", reasons)

    def test_missing_autostart_does_not_crash_grouping(self) -> None:
        log = make_log(
            2,
            [{"cntlog": "1"}],
            auto_start=2,
            optimal_trace=2,
        )
        del log.params["autoStart"]

        groups = group_auto_start_sets([log])

        self.assertEqual(groups, [[log]])


if __name__ == "__main__":
    unittest.main()
