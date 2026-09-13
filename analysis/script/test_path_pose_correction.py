#!/usr/bin/env python3
"""経路制御Version 13のライン観測点・勾配補正式の回帰テスト。"""

from __future__ import annotations

import math
import unittest
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    heading_deg: float


def wrap_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def sensor_world(pose: Pose, lateral_mm: float, forward_mm: float) -> tuple[float, float]:
    heading_rad = math.radians(pose.heading_deg)
    heading_sin = math.sin(heading_rad)
    heading_cos = math.cos(heading_rad)
    return (
        pose.x_mm + lateral_mm * heading_cos + forward_mm * heading_sin,
        pose.y_mm - lateral_mm * heading_sin + forward_mm * heading_cos,
    )


def error_energy(pose: Pose, lateral_mm: float, forward_mm: float,
                 match_x_mm: float, match_y_mm: float) -> float:
    sensor_x, sensor_y = sensor_world(pose, lateral_mm, forward_mm)
    return 0.5 * ((sensor_x - match_x_mm) ** 2 + (sensor_y - match_y_mm) ** 2)


def correct_pose(
    pose: Pose,
    lateral_mm: float,
    forward_mm: float,
    match_x_mm: float,
    match_y_mm: float,
    line_alpha_x1000: int,
    line_theta_gain_x1e9: int,
    *,
    corridor_blocked: bool = False,
    residual_limit_mm: float = 30.0,
    position_limit_mm: float = 1.0,
    heading_limit_deg: float = 0.1,
) -> Pose:
    sensor_x, sensor_y = sensor_world(pose, lateral_mm, forward_mm)
    residual_x = sensor_x - match_x_mm
    residual_y = sensor_y - match_y_mm
    residual_mm = math.hypot(residual_x, residual_y)
    if corridor_blocked or residual_mm > residual_limit_mm:
        return pose

    alpha = line_alpha_x1000 * 0.001
    correction_x = -alpha * residual_x
    correction_y = -alpha * residual_y
    heading_rad = math.radians(pose.heading_deg)
    derivative_x = -lateral_mm * math.sin(heading_rad) + forward_mm * math.cos(heading_rad)
    derivative_y = -lateral_mm * math.cos(heading_rad) - forward_mm * math.sin(heading_rad)
    theta_gradient = residual_x * derivative_x + residual_y * derivative_y
    theta_gain = line_theta_gain_x1e9 * 1.0e-9
    heading_correction_deg = math.degrees(-theta_gain * theta_gradient)
    if math.hypot(correction_x, correction_y) > position_limit_mm:
        return pose
    if abs(heading_correction_deg) > heading_limit_deg:
        return pose
    return Pose(
        pose.x_mm + correction_x,
        pose.y_mm + correction_y,
        wrap_deg(pose.heading_deg + heading_correction_deg),
    )


class PathPoseCorrectionTest(unittest.TestCase):
    def assert_point_close(self, actual: tuple[float, float], expected: tuple[float, float]) -> None:
        self.assertAlmostEqual(actual[0], expected[0], places=6)
        self.assertAlmostEqual(actual[1], expected[1], places=6)

    def test_sensor_world_uses_clockwise_positive_heading(self) -> None:
        self.assert_point_close(sensor_world(Pose(0.0, 0.0, 0.0), 10.0, 95.0), (10.0, 95.0))
        self.assert_point_close(sensor_world(Pose(0.0, 0.0, 90.0), 10.0, 95.0), (95.0, -10.0))
        self.assert_point_close(sensor_world(Pose(0.0, 0.0, -90.0), 10.0, 95.0), (-95.0, 10.0))

    def test_left_right_sensor_observations_are_symmetric(self) -> None:
        left = sensor_world(Pose(0.0, 0.0, 0.0), -41.70, 76.07)
        right = sensor_world(Pose(0.0, 0.0, 0.0), 41.70, 76.07)
        self.assertAlmostEqual(left[0], -right[0], places=6)
        self.assertAlmostEqual(left[1], right[1], places=6)

        left_corrected = correct_pose(Pose(0.0, 0.0, 0.0), -41.70, 76.07, -40.0, 76.07, 10, 0)
        right_corrected = correct_pose(Pose(0.0, 0.0, 0.0), 41.70, 76.07, 40.0, 76.07, 10, 0)
        self.assertAlmostEqual(left_corrected.x_mm, -right_corrected.x_mm, places=6)
        self.assertAlmostEqual(left_corrected.y_mm, right_corrected.y_mm, places=6)

    def test_translation_correction_direction_at_cardinal_headings(self) -> None:
        for heading_deg in (0.0, 90.0, -90.0):
            with self.subTest(heading_deg=heading_deg):
                pose = Pose(0.0, 0.0, heading_deg)
                sensor_x, sensor_y = sensor_world(pose, 10.0, 95.0)
                corrected = correct_pose(
                    pose, 10.0, 95.0, sensor_x + 4.0, sensor_y - 3.0, 10, 0
                )
                self.assertGreater(corrected.x_mm, pose.x_mm)
                self.assertLess(corrected.y_mm, pose.y_mm)

    def test_one_gradient_step_reduces_error_energy(self) -> None:
        pose = Pose(10.0, 0.0, 0.0)
        before = error_energy(pose, 0.0, 95.0, 0.0, 95.0)
        corrected = correct_pose(pose, 0.0, 95.0, 0.0, 95.0, 10, 100)
        after = error_energy(corrected, 0.0, 95.0, 0.0, 95.0)
        self.assertLess(after, before)

    def test_zero_heading_gain_does_not_change_heading(self) -> None:
        pose = Pose(10.0, 0.0, 12.0)
        corrected = correct_pose(pose, 0.0, 95.0, 0.0, 95.0, 10, 0)
        self.assertEqual(corrected.heading_deg, pose.heading_deg)

    def test_residual_limit_rejects_correction(self) -> None:
        pose = Pose(31.0, 0.0, 0.0)
        self.assertEqual(correct_pose(pose, 0.0, 95.0, 0.0, 95.0, 10, 100), pose)

    def test_per_cycle_position_limit_rejects_correction(self) -> None:
        pose = Pose(15.0, 0.0, 0.0)
        self.assertEqual(correct_pose(pose, 0.0, 95.0, 0.0, 95.0, 100, 0), pose)

    def test_corridor_rejects_translation_and_heading_correction(self) -> None:
        pose = Pose(10.0, 0.0, 0.0)
        corrected = correct_pose(
            pose, 0.0, 95.0, 0.0, 95.0, 10, 100, corridor_blocked=True
        )
        self.assertEqual(corrected, pose)


if __name__ == "__main__":
    unittest.main()
