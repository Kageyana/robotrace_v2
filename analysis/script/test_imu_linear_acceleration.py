#!/usr/bin/env python3
"""静止時重力基準を除去したIMU線形加速度APIの計算契約を検証する。"""

from __future__ import annotations

import math
import unittest


GRAVITY_MPS2 = 9.80665


def linear_acceleration(
    sensor_g: tuple[float, float, float],
    gravity_g: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        (sensor_g[index] - gravity_g[index]) * GRAVITY_MPS2
        for index in range(3)
    )


def forward_acceleration(
    sensor_g: tuple[float, float, float],
    gravity_g: tuple[float, float, float],
) -> float:
    """IMU_GetForwardAccelerationMps2()のY軸互換ラッパー契約。"""
    return linear_acceleration(sensor_g, gravity_g)[1]


class ImuLinearAccelerationTests(unittest.TestCase):
    def test_gravity_reference_is_removed_and_converted_to_mps2(self) -> None:
        values = linear_acceleration((0.12, -0.34, 1.08), (0.02, -0.04, 1.00))
        self.assertAlmostEqual(values[0], 0.1 * GRAVITY_MPS2)
        self.assertAlmostEqual(values[1], -0.3 * GRAVITY_MPS2)
        self.assertAlmostEqual(values[2], 0.08 * GRAVITY_MPS2)
        self.assertTrue(all(math.isfinite(value) for value in values))

    def test_forward_acceleration_is_the_same_as_y_axis_api(self) -> None:
        sensor = (0.01, 0.27, 1.02)
        gravity = (0.01, 0.02, 1.00)
        linear = linear_acceleration(sensor, gravity)
        forward = forward_acceleration(sensor, gravity)
        self.assertEqual(forward, linear[1])


if __name__ == "__main__":
    unittest.main()
