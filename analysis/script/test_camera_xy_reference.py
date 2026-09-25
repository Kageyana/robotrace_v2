"""上方カメラ独立XY基準の座標・符号・同期・欠測回帰テスト。"""

from __future__ import annotations

import csv
import json
import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from analysis.script.camera_xy_reference import (
    CalibrationPoint, LogTimeline, _log_time_at_event, _read_log, _time_at_pulse,
    _video_fps_gate, build_reference, calibration_report, fit_homography,
    register_ground_frame, robot_origin_pose, transform_point,
)


class CameraXYReferenceTests(unittest.TestCase):
    def test_perspective_homography_uses_independent_check_points(self) -> None:
        expected = np.array([
            [1.12, 0.04, 45.0],
            [-0.03, 0.91, -22.0],
            [0.00012, -0.00008, 1.0],
        ])
        pixel_fit = np.array([
            [0, 0], [1000, 0], [1000, 800], [0, 800],
            [500, 0], [1000, 400], [500, 800], [0, 400],
        ], dtype=float)
        floor_fit = np.array([transform_point(expected, *point) for point in pixel_fit])
        pixel_check = np.array([[250, 200], [750, 200], [750, 600], [250, 600]], dtype=float)
        floor_check = np.array([transform_point(expected, *point) for point in pixel_check])
        points = [CalibrationPoint(f"f{i}", *pixel, *floor, "fit")
                  for i, (pixel, floor) in enumerate(zip(pixel_fit, floor_fit))]
        points += [CalibrationPoint(f"c{i}", *pixel, *floor, "check")
                   for i, (pixel, floor) in enumerate(zip(pixel_check, floor_check))]
        matrix, report = calibration_report(points)
        self.assertTrue(report["passed"])
        self.assertLess(report["check_max_mm"], 1e-7)
        for pixel, expected_floor in zip(pixel_check, floor_check):
            actual_floor = transform_point(matrix, *pixel)
            np.testing.assert_allclose(actual_floor, expected_floor, atol=1e-7)

    def test_marker_order_and_heading_sign_are_explicit(self) -> None:
        geometry = {
            "marker_midpoint_to_origin_right_mm": 0.0,
            "marker_midpoint_to_origin_forward_mm": 0.0,
        }
        _, _, straight, baseline = robot_origin_pose((0, 50), (0, -50), geometry)
        _, _, clockwise, _ = robot_origin_pose((50, 0), (-50, 0), geometry)
        _, _, swapped, _ = robot_origin_pose((-50, 0), (50, 0), geometry)
        self.assertAlmostEqual(straight, 0.0)
        self.assertAlmostEqual(clockwise, 90.0)
        self.assertAlmostEqual(abs(swapped - clockwise), 180.0)
        self.assertAlmostEqual(baseline, 100.0)

    def test_measured_marker_lateral_offset_corrects_body_heading(self) -> None:
        forward = math.sqrt(100.0**2 - 10.0**2)
        front = (5.0, forward / 2.0)
        rear = (-5.0, -forward / 2.0)
        geometry = {
            "marker_baseline_mm": 100.0,
            "front_marker_lateral_offset_mm": 10.0,
            "marker_midpoint_to_origin_right_mm": 0.0,
            "marker_midpoint_to_origin_forward_mm": 0.0,
        }
        _, _, heading, baseline = robot_origin_pose(front, rear, geometry)
        self.assertAlmostEqual(heading, 0.0)
        self.assertAlmostEqual(baseline, 100.0)
        geometry["front_marker_lateral_offset_mm"] = -10.0
        _, _, opposite_heading, _ = robot_origin_pose((-5.0, forward / 2.0),
                                                       (5.0, -forward / 2.0), geometry)
        self.assertAlmostEqual(opposite_heading, 0.0)

    def test_frame_registration_uses_separate_ground_check_points(self) -> None:
        ground_h = np.array([[1.1, 0.02, 20.0], [-0.01, 0.9, -40.0],
                             [0.0001, -0.00005, 1.0]])
        current_to_initial = np.array([[1.01, -0.005, -9.0],
                                       [0.008, 0.99, 7.0],
                                       [0.00001, -0.00002, 1.0]])
        current_fixed = [(100, 100), (900, 100), (900, 700), (100, 700)]
        current_check = [(300, 250), (650, 200), (700, 550), (250, 600)]
        initial_fixed = [transform_point(current_to_initial, *point)
                         for point in current_fixed]
        initial_check = [transform_point(current_to_initial, *point)
                         for point in current_check]
        warp, maximum = register_ground_frame(
            ground_h, current_fixed, initial_fixed, current_check, initial_check)
        self.assertLess(maximum, 1e-8)
        np.testing.assert_allclose(transform_point(warp, 500, 400),
                                   transform_point(current_to_initial, 500, 400), atol=1e-8)
        displaced = list(initial_check)
        displaced[0] = (displaced[0][0] + 10.0, displaced[0][1])
        _, erroneous = register_ground_frame(
            ground_h, current_fixed, initial_fixed, current_check, displaced)
        self.assertGreater(erroneous, 5.0)

    def test_isolated_frame_drop_keeps_nominal_60fps_and_gap_visible(self) -> None:
        probe = {"nominal_stream_fps": 60.0, "stream_average_fps": 59.92,
                 "measured_average_fps": 59.92, "median_frame_interval_fps": 60.0,
                 "frame_count": 1803, "long_frame_gaps": 1,
                 "max_frame_gap_ms": 50.0, "nominal_frame_period_ms": 1000.0 / 60.0}
        self.assertFalse(_video_fps_gate(probe, False)[0])
        self.assertTrue(_video_fps_gate(probe, True)[0])
        probe["long_frame_gaps"] = 5
        self.assertFalse(_video_fps_gate(probe, True)[0])

    def test_cntlog_wrap_and_goal_pulse_interpolation(self) -> None:
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            log_path = Path(directory) / "wrap.csv"
            metadata = self._schema10_metadata(
                row_count=5, goal_pulse=3500, start_delay=20,
            )
            rows = [
                [10, 1000, 0, 0, 0, 0],
                [32760, 2000, 0, 0, 0, 0],
                [65520, 3000, 0, 0, 0, 0],
                [4, 4000, 0, 0, 0, 0],
                [14, 5000, 0, 0, 0, 0],
            ]
            self._write_log(log_path, metadata, rows)
            timeline = _read_log(log_path)
            self.assertEqual(timeline.time_ms, [10, 32760, 65520, 65540, 65550])
            # goal pulse 2500 is halfway between rows at 65520ms and 65540ms.
            self.assertEqual(_time_at_pulse(timeline, 3500), 65530.0)
            offset, sync = _log_time_at_event(timeline, {
                "start_sensor_onset_video_ms": 100.0,
                "goal_sensor_onset_video_ms": 65650.0,
            })
            self.assertEqual(offset, -120.0)
            self.assertEqual(sync["goal_event_log_time_ms"], 65530.0)
            self.assertEqual(sync["goal_sync_residual_ms"], 0.0)
            _, shifted = _log_time_at_event(timeline, {
                "start_sensor_onset_video_ms": 100.0,
                "goal_sensor_onset_video_ms": 65660.0,
            })
            self.assertEqual(shifted["goal_sync_residual_ms"], -10.0)

    @unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"),
                         "ffmpeg/ffprobe are required for the synthetic video test")
    def test_synthetic_video_creates_synced_reference_without_filling_tracking_gaps(self) -> None:
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            video = root / "synthetic.mkv"
            log = root / "12656.csv"
            calibration = root / "calibration.csv"
            setup = root / "camera.json"
            events = root / "events.json"
            output = root / "output"
            self._write_synthetic_video(root, video)
            self._write_synthetic_calibration(calibration, root)
            self._write_synthetic_setup(setup)
            self._write_synthetic_log(log)
            events.write_text(json.dumps({
                "start_sensor_onset_video_ms": 100.0,
                "goal_sensor_onset_video_ms": 1800.0,
                "event_definition": "right marker sensor at stripe leading edge",
            }), encoding="utf-8")

            diagnostics = build_reference(video, log, calibration, setup, events, output)
            self.assertTrue(diagnostics["reference_valid"])
            self.assertLessEqual(diagnostics["calibration"]["check_max_mm"], 5.0)
            self.assertLessEqual(abs(diagnostics["synchronization"]["goal_sync_residual_ms"]),
                                 diagnostics["synchronization"]["half_frame_limit_ms"])
            with (output / "reference.csv").open(encoding="utf-8", newline="") as source:
                reference = list(csv.DictReader(source))
            missed = [row for row in reference if 30 <= int(row["frame_index"]) <= 32]
            self.assertEqual(len(missed), 3)
            self.assertTrue(all(row["track_valid"] == "0" for row in missed))
            self.assertTrue(all(row["x_mm"] == "" and row["y_mm"] == "" and
                                row["heading_cw_deg"] == "" for row in missed))
            observed_turn = [float(row["heading_cw_deg"]) for row in reference
                             if row["track_valid"] == "1"]
            self.assertGreater(max(observed_turn), 20.0)
            report = json.loads((output / "diagnostics.json").read_text(encoding="utf-8"))
            self.assertFalse(report["tracking"]["position_rows_are_interpolated"])
            self.assertTrue(report["tracking"]["stationary_pose_jitter"]["sufficient_samples"])
            self.assertAlmostEqual(
                report["tracking"]["stationary_pose_jitter"]["position_rms_mm"], 0.0)

            events.write_text(json.dumps({
                "start_sensor_onset_video_ms": 100.0,
                "goal_sensor_onset_video_ms": 1820.0,
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "synchronization residual"):
                build_reference(video, log, calibration, setup, events, output)
            self.assertFalse((output / "reference.csv").exists())

    @staticmethod
    def _schema10_metadata(row_count: int, goal_pulse: int, start_delay: int) -> list[str]:
        values = {
            "logSchemaVersion": 10, "optimalTrace": 0.0, "closureValid": 1,
            "closureReason": 0, "emcStop": 0, "logOverflowFinal": 0,
            "dbgOverflowFinal": 0, "distanceScaleVerified": 1,
            "encoderPulsePerMeter": 58019, "logRecordSizeBytes": 36,
            "imuCalibrationValid": 1, "imuCalibrationSamples": 100,
            "imuCalibrationReadErrors": 0, "gyroSampleFault": 0,
            "encoderIntervalFault": 0, "timing.startResetMeasured": 1,
            "timing.startResetDelay_ms": start_delay,
            "goalMarkerOnsetValid": 1, "goalMarkerOnset_p": goal_pulse,
            "logExpectedRows": row_count, "batteryVoltage_V": 8.0,
            "gitCommit": "synthetic", "branch": "test", "fwVersion": "synthetic",
        }
        return [f"{key}={value}" for key, value in values.items()]

    @classmethod
    def _write_log(cls, path: Path, metadata: list[str], rows: list[list[float]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(metadata)
            writer.writerow(["cntlog", "encTotalOptimal", "gyroVal_Z", "x", "y", "encCurrentN"])
            for row in rows:
                writer.writerow(row)

    @classmethod
    def _write_synthetic_log(cls, path: Path) -> None:
        rows = []
        pulse_per_ms = 58019.0 * 0.3 / 1000.0
        goal_time_ms = 1680.0
        goal_pulse = round(goal_time_ms * pulse_per_ms)
        for time_ms in range(10, 1901, 10):
            rows.append([time_ms, round(time_ms * pulse_per_ms), 0.0, 0.0,
                         0.3 * time_ms, 0])
        cls._write_log(path, cls._schema10_metadata(len(rows), goal_pulse, 20), rows)

    @staticmethod
    def _write_synthetic_calibration(path: Path, root: Path) -> None:
        fit_xy = [(20, 20), (300, 20), (300, 220), (20, 220),
                  (160, 20), (300, 120), (160, 220), (20, 120)]
        check_xy = [(70, 60), (250, 60), (250, 180), (70, 180)]
        with path.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(["id", "pixel_x", "pixel_y", "floor_x_mm", "floor_y_mm", "split"])
            for index, (x, y) in enumerate(fit_xy):
                writer.writerow([f"f{index}", x, y, x * 10, -y * 10, "fit"])
            for index, (x, y) in enumerate(check_xy):
                writer.writerow([f"c{index}", x, y, x * 10, -y * 10, "check"])

    @staticmethod
    def _write_synthetic_setup(path: Path) -> None:
        setup = {
            "markers": {
                "front": {"seed_pixel": [160, 146], "hue_min": 110, "hue_max": 145,
                          "saturation_min": 180, "value_min": 180, "search_radius_px": 24,
                          "area_min_px": 20, "area_max_px": 200},
                "rear": {"seed_pixel": [160, 156], "hue_min": 245, "hue_max": 12,
                         "saturation_min": 180, "value_min": 180, "search_radius_px": 24,
                         "area_min_px": 20, "area_max_px": 200},
            },
            "fixed_reference_ids": ["c0", "c1", "c2", "c3"],
            "fixed_reference_color": {"hue_min": 35, "hue_max": 55,
                                      "saturation_min": 180, "value_min": 180,
                                      "search_radius_px": 10, "area_min_px": 20,
                                      "area_max_px": 200},
            "geometry": {
                "marker_baseline_mm": 100.0,
                "marker_midpoint_to_origin_right_mm": 0.0,
                "marker_midpoint_to_origin_forward_mm": 0.0,
                "right_marker_sensor_right_mm": 55.0,
                "right_marker_sensor_forward_mm": -20.0,
                "marker_height_mm": 35.0,
            },
            "max_marker_baseline_error_mm": 10.0,
            "max_camera_motion_mm": 5.0,
            "expected_start_heading_cw_deg": 0.0,
            "start_heading_tolerance_deg": 20.0,
            "stationary_check_video_interval_ms": [0.0, 100.0],
        }
        path.write_text(json.dumps(setup, indent=2), encoding="utf-8")

    @staticmethod
    def _write_synthetic_video(root: Path, video: Path) -> None:
        frames = root / "frames"
        frames.mkdir()
        fit_xy = [(20, 20), (300, 20), (300, 220), (20, 220),
                  (160, 20), (300, 120), (160, 220), (20, 120)]
        check_xy = [(70, 60), (250, 60), (250, 180), (70, 180)]
        front_missing = set(range(30, 33))
        for frame_index in range(120):
            image = Image.new("RGB", (320, 240), (15, 15, 15))
            draw = ImageDraw.Draw(image)
            for x, y in check_xy:
                draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=(255, 255, 0))

            time_s = frame_index / 60.0
            heading = math.radians(max(0.0, min(45.0, (time_s - 0.5) * 35.0)))
            center_x = 160.0
            center_y = 154.0 - max(0.0, time_s - 0.12) * 28.0
            forward_x, forward_y = math.sin(heading), -math.cos(heading)
            front = (center_x + forward_x * 5.0, center_y + forward_y * 5.0)
            rear = (center_x - forward_x * 5.0, center_y - forward_y * 5.0)
            if frame_index not in front_missing:
                draw.ellipse((front[0] - 4, front[1] - 4, front[0] + 4,
                              front[1] + 4), fill=(0, 255, 255))
            draw.ellipse((rear[0] - 4, rear[1] - 4, rear[0] + 4,
                          rear[1] + 4), fill=(255, 0, 0))
            image.save(frames / f"frame_{frame_index:04d}.png")
        subprocess.run([
            shutil.which("ffmpeg") or "ffmpeg", "-v", "error", "-y", "-framerate", "60",
            "-i", str(frames / "frame_%04d.png"), "-frames:v", "120", "-c:v", "png",
            "-pix_fmt", "rgb24", str(video),
        ], check=True, capture_output=True)


if __name__ == "__main__":
    unittest.main()
